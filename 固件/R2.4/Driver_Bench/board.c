/* R2.4 板级驱动。TIMER0 决定采样时刻，两个 SPI 分别读写功率驱动和编码器，USART0 用 DMA 收发。 */
#include "board.h"
#include "gd32f30x.h"
#include "wave.h"
#include <string.h>

#define DRVOFF GPIO_PIN_11 /* PB11 高电平禁止功率桥输出。 */
#define CTRL2_TIMING 0x70u /* 保留位、SDO 推挽、125 V/µs 和普通 6PWM，保证低侧采样建立时间。 */
#define NSLEEP GPIO_PIN_10 /* PB10 高电平唤醒驱动芯片，此时可保持 DRVOFF 为高并配置 SPI。 */
static volatile uint8_t rx_ring[256]; /* DMA 写入的串口接收环形缓冲区，主循环按字节取命令。 */
static bool running; /* 上一拍已开 PWM，用来识别运行期间的硬件刹车。 */
#ifndef AXIS_WAVE_OUTPUT
static bool hiz_guard(void);
#endif

/* cycles 为 CPU 时钟数，120 MHz 下 60 个周期对应 0.5 µs，用于满足 SPI 片选时序等短延时。 */
static void delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

/* 立即撤销驱动允许及互补 PWM 主输出。硬件上 PB11 另有 R32 上拉。 */
void bridge_off(void)
{
    GPIO_BOP(GPIOB) = DRVOFF;
    TIMER_CCHP(TIMER0) &= ~TIMER_CCHP_POEN;
    running = false;
}

#ifndef AXIS_WAVE_OUTPUT
/* 关桥前只读本地寄存器。TP8 即使已经回低，BRK 仍可记录两个采样中断之间的刹车。 */
const bridge_trip_t* board_note_trip(const char* site)
{
    static bridge_trip_t first;
    if(!first.site)
    {
        uint32_t flags = TIMER_INTF(TIMER0);
        first.timer_control = (uint16_t)TIMER_CCHP(TIMER0);
        first.port_a = (uint16_t)GPIO_ISTAT(GPIOA);
        first.port_b = (uint16_t)GPIO_ISTAT(GPIOB);
        first.running = running;
        first.brk = (flags & TIMER_FLAG_BRK) != 0;
        first.pwm = (first.timer_control & TIMER_CCHP_POEN) != 0;
        first.tp8 = (first.port_a & GPIO_PIN_6) != 0;
        first.off = (first.port_b & DRVOFF) != 0;
        first.site = site;
    }
    return &first;
}
#endif

static bool bridge_reject(const char* site)
{
#ifndef AXIS_WAVE_OUTPUT
    board_note_trip(site);
#else
    (void)site;
#endif
    bridge_off();
    return false;
}

/* 仅在从禁能切换到运行时设置 POEN。运行期间被 BKIN 清除的 POEN 绝不自动恢复。 */
bool bridge_set(bool enable, const float duty[3])
{
    for(unsigned i = 0; enable && i < 3; ++i)
    {
        if(!(duty[i] >= 0.35f && duty[i] <= MAX_DUTY_PERCENT / 100.0f))
        {
            return bridge_reject("DUTY");
        }
    }
    TIMER_CH0CV(TIMER0) = (uint32_t)(duty[0] * TOP);
    TIMER_CH1CV(TIMER0) = (uint32_t)(duty[1] * TOP);
    TIMER_CH2CV(TIMER0) = (uint32_t)(duty[2] * TOP);
    if(!enable)
    {
        bridge_off();
        return true;
    }
    if(GPIO_ISTAT(GPIOA) & GPIO_PIN_6)
    {
        return bridge_reject(running ? "RUN_TP8" : "START_TP8");
    }
    uint32_t enable_start = 0;
    if(!running)
    {
#ifdef AXIS_WAVE_OUTPUT
        timer_flag_clear(TIMER0, TIMER_FLAG_BRK);
#else
        if(board_is_off())
            return bridge_reject("START_DRVOFF");
        if(!hiz_guard())
            return bridge_reject("START_HIZ");
        if(TIMER_INTF(TIMER0) & TIMER_FLAG_BRK)
            return bridge_reject("START_BRK");
#endif
        enable_start = DWT->CYCCNT;
        TIMER_CCHP(TIMER0) |= TIMER_CCHP_POEN;
    }
    /* 实板首次写入后可先读到 0、随后读到 1。期间只读回，并在每次读回时检查硬件保护。 */
    for(;;)
    {
        bool pwm_enabled = (TIMER_CCHP(TIMER0) & TIMER_CCHP_POEN) != 0;
        if(TIMER_INTF(TIMER0) & TIMER_FLAG_BRK)
            return bridge_reject(running ? "RUN_BRK" : "ENABLE_BRK");
        if(GPIO_ISTAT(GPIOA) & GPIO_PIN_6)
            return bridge_reject(running ? "RUN_TP8" : "ENABLE_TP8");
        if(pwm_enabled)
            break;
        if(running)
            return bridge_reject("RUN_POEN");
        if(DWT->CYCCNT - enable_start >= POEN_CONFIRM_TICKS)
            return bridge_reject("ENABLE_TIMEOUT");
    }
    GPIO_BC(GPIOB) = DRVOFF;
    running = true;
    return true;
}

/* 两条独立 SPI 均为 16 位、模式 1。每帧单独拉片选，超时后返回错误。 */
static bool transfer(uint32_t spi, uint16_t tx, uint16_t* rx)
{
    uint32_t port = spi == SPI0 ? GPIOA : GPIOB;
    uint32_t cs = spi == SPI0 ? GPIO_PIN_15 : GPIO_PIN_12;
    GPIO_BC(port) = cs;
    delay_cycles(60);
    SPI_DATA(spi) = tx;
    uint32_t start = DWT->CYCCNT;
    while(!(SPI_STAT(spi) & SPI_STAT_RBNE) || (SPI_STAT(spi) & SPI_STAT_TRANS))
    {
        if(DWT->CYCCNT - start > 1200u)
        {
            GPIO_BOP(port) = cs;
            return false;
        }
    }
    *rx = (uint16_t)SPI_DATA(spi);
    delay_cycles(60);
    GPIO_BOP(port) = cs;
    delay_cycles(60);
    return true;
}

/* DRV8316C 一帧内完成读写。位 15 选择读写，位 14～9 是地址，位 8 是偶校验，低八位是数据。 */
static bool drv_reg(bool read, unsigned addr, unsigned value, uint16_t* result)
{
    uint16_t frame = (uint16_t)((read ? 0x8000u : 0u) | (addr << 9) | value);
    uint16_t rx;
    if(!transfer(SPI0, spi_even_parity(frame, 0x0100), &rx))
    {
        return false;
    }
    *result = rx & 255u;
    return true;
}

/* AS5048A 的第二帧才返回第一帧所请求的寄存器；NOP 本身也占一个完整片选周期。 */
static bool enc_read(unsigned address, uint16_t* value)
{
    uint16_t rx;
    if(!transfer(SPI1, spi_even_parity(0x4000u | address, 0x8000), &rx) || !transfer(SPI1, 0, &rx) ||
        !encoder_frame_ok(rx))
    {
        return false;
    }
    *value = rx & 0x3fffu;
    return true;
}

/* R2.4 用 TIMER0 部分重映射。PA6 接经 Q1 反相的高有效故障，始终保留 SWD。 */
static void setup_pins(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    GPIO_BOP(GPIOB) = DRVOFF | GPIO_PIN_12;
    GPIO_BC(GPIOB) = NSLEEP;
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DRVOFF | NSLEEP | GPIO_PIN_12);
    uint32_t remap = GPIO_SPI0_REMAP | AFIO_PCF0_USART0_REMAP | PCF0_TIMER0_REMAP(1) | PCF0_SWJ_CFG(2);
    AFIO_PCF0 = remap; /* 一次写入引脚重映射与 SWJ_CFG，避免读回未定义的调试端口配置位。 */
    GPIO_BOP(GPIOA) = GPIO_PIN_15;
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_12);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_2MHZ,
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_2MHZ, GPIO_PIN_6);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_13 | GPIO_PIN_15);
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_14);
}

/* 中心对齐六路互补 PWM，死区 120 个 120 MHz 时钟。CH3 仅在内部产生每周期一次的 ADC 触发上升沿。 */
static void setup_pwm(void)
{
    rcu_periph_clock_enable(RCU_TIMER0);
    TIMER_CTL0(TIMER0) = TIMER_COUNTER_CENTER_UP | TIMER_CTL0_ARSE;
    TIMER_CAR(TIMER0) = TOP;
    TIMER_CREP(TIMER0) = 3; /* 4 个半周期在谷底提交一次三相占空比。 */
    uint32_t pwm = TIMER_OC_MODE_PWM0 | TIMER_OC_SHADOW_ENABLE;
    TIMER_CHCTL0(TIMER0) = pwm | (pwm << 8);
    TIMER_CHCTL1(TIMER0) = pwm | (TIMER_OC_MODE_PWM1 << 8);
    TIMER_CHCTL2(TIMER0) = TIMER_CHCTL2_CH0EN | TIMER_CHCTL2_CH0NEN | TIMER_CHCTL2_CH1EN |
        TIMER_CHCTL2_CH1NEN | TIMER_CHCTL2_CH2EN | TIMER_CHCTL2_CH2NEN;
    TIMER_CH0CV(TIMER0) = TIMER_CH1CV(TIMER0) = TIMER_CH2CV(TIMER0) = TOP / 2u;
    TIMER_CH3CV(TIMER0) = TOP - TRIGGER_LEAD;
    TIMER_CTL1(TIMER0) = TIMER_TRI_OUT_SRC_O3CPRE;
    TIMER_CCHP(TIMER0) = TIMER_CCHP_BRKEN | TIMER_CCHP_BRKP | TIMER_CCHP_IOS | 120u;
    TIMER_SWEVG(TIMER0) = TIMER_SWEVG_UPG;
    /* TIMER1 以内触发 ITI0 计 TIMER0 的 CH3 上升沿，首次溢出在第一载波，随后隔一载波。 */
    rcu_periph_clock_enable(RCU_TIMER1);
    TIMER_CAR(TIMER1) = 1;
    TIMER_SWEVG(TIMER1) = TIMER_SWEVG_UPG;
    TIMER_CNT(TIMER1) = 1;
    TIMER_SMCFG(TIMER1) = TIMER_SMCFG_TRGSEL_ITI0 | TIMER_SLAVE_MODE_EXTERNAL0;
    TIMER_CTL1(TIMER1) = TIMER_TRI_OUT_SRC_UPDATE;
}

/* 拉高 nSLEEP 后等待 10 ms，再解锁配置寄存器、写入模式和增益、重新上锁并核对读回值。 */
static bool setup_driver(raw_t* r)
{
    uint16_t unused;
    GPIO_BOP(GPIOB) = NSLEEP;
    delay_cycles(1200000);
    /* 保留 CTRL2 的保留位与 SDO 输出方式，选择普通 6PWM、清上电标志；CTRL5 选 0.30 V/A。 */
    if(!drv_reg(true, 4, 0, &r->drv_ctrl2) || !drv_reg(true, 7, 0, &r->drv_ctrl5))
    {
        return false;
    }
    if((r->drv_ctrl2 & 0x60u) != 0x60u)
    {
        return false;
    }
    if(!drv_reg(false, 3, 3, &unused) || !drv_reg(false, 4, (r->drv_ctrl2 & ~0x1fu) | 0x11u, &unused) ||
        !drv_reg(false, 7, (r->drv_ctrl5 & ~3u) | 1u, &unused) || !drv_reg(false, 3, 6, &unused))
    {
        return false;
    }
    return drv_reg(true, 4, 0, &r->drv_ctrl2) && drv_reg(true, 7, 0, &r->drv_ctrl5) &&
        (r->drv_ctrl2 & 0x7eu) == CTRL2_TIMING && (r->drv_ctrl5 & 3u) == 1u;
}

/* 保持 DRVOFF，建立串口 DMA 和两个 SPI 接口，再唤醒驱动芯片并读回配置。 */
bool board_init(raw_t* raw)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    setup_pins();
    setup_pwm();
    bridge_off();
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_DMA0);
    usart_baudrate_set(USART0, 921600);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    USART_CTL2(USART0) |= USART_CTL2_DENT | USART_CTL2_DENR;
    DMA_CHPADDR(DMA0, DMA_CH4) = (uint32_t)&USART_DATA(USART0);
    DMA_CHMADDR(DMA0, DMA_CH4) = (uint32_t)rx_ring;
    DMA_CHCNT(DMA0, DMA_CH4) = sizeof(rx_ring);
    DMA_CHCTL(DMA0, DMA_CH4) = DMA_CHXCTL_CMEN | DMA_CHXCTL_MNAGA | DMA_CHXCTL_CHEN;
    usart_enable(USART0);
    DMA_CHPADDR(DMA0, DMA_CH3) = (uint32_t)&USART_DATA(USART0);
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_SPI1);
    uint32_t mode =
        SPI_CTL0_MSTMOD | SPI_CTL0_SWNSS | SPI_CTL0_SWNSSEN | SPI_CTL0_FF16 | SPI_CTL0_CKPH | SPI_CTL0_SPIEN;
    SPI_CTL0(SPI0) = mode | SPI_PSC_32;
    SPI_CTL0(SPI1) = mode | SPI_PSC_16;
    return setup_driver(raw);
}

/* ADC0 顺序采三相和母线，ADC1 在后台连续采 PA4 的半参考电压。两路 ADC 各自校准。 */
void board_start_sampling(void)
{
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 4);
    for(unsigned i = 0; i < 4; ++i)
    {
        adc_inserted_channel_config(ADC0, i, i, i == 3 ? ADC_SAMPLETIME_55POINT5 : ADC_SAMPLETIME_13POINT5);
    }
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T1_TRGO);
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_routine_channel_config(ADC1, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_55POINT5);
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
    adc_external_trigger_source_config(ADC1, ADC_ROUTINE_CHANNEL, ADC0_1_2_EXTTRIG_ROUTINE_NONE);
    adc_external_trigger_config(ADC1, ADC_ROUTINE_CHANNEL, ENABLE);
    adc_enable(ADC0);
    adc_enable(ADC1);
    delay_cycles(120000);
    adc_calibration_enable(ADC0);
    adc_calibration_enable(ADC1);
    adc_software_trigger_enable(ADC1, ADC_ROUTINE_CHANNEL);
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
    NVIC_EnableIRQ(ADC0_1_IRQn);
    timer_enable(TIMER1);
    timer_enable(TIMER0);
}

/* 每拍采集 ADC 和角度，将磁场、供电参考和驱动读回异常传递给控制状态机。 */
void board_sample(raw_t* r, sample_t* s, bool diagnostics)
{
    /* slot 是 100 拍诊断周期内的序号。failed 的低五位分别跟踪三个状态寄存器和两个控制寄存器。
     * 初始五位均置位，每项检查通过才清除对应位，防止启动时把未读过的寄存器当作正常。 */
    static unsigned slot, failed = 0x1fu;
    for(unsigned i = 0; i < 4; ++i)
    {
        r->adc[i] = adc_inserted_data_read(ADC0, i);
    }
    r->mid = adc_routine_data_read(ADC1);
    /* R5 为 100 kΩ、R6 为 10 kΩ，PA3 读到母线的 1/11，乘 11 还原桥上的电压。 */
    s->bus = r->adc[3] * (3.3f * 11.0f / 4095.0f);
    if(diagnostics)
    {
        /* 位 8 表示偏移补偿完成，位 9～11 报告运算溢出或磁场强弱异常。 */
        r->enc_diag_ok = enc_read(0x3ffdu, &r->enc_diag);
    }
    /* 每 100 个控制周期分散读五个寄存器，避免慢速诊断挤占同一次采样。 */
    if(slot >= 1 && slot <= 5)
    {
        unsigned index = slot - 1;
        bool ok;
        if(index < 3)
        {
            ok = drv_reg(true, index, 0, &r->drv_status[index]);
        }
        else if(index == 3)
        {
            ok = drv_reg(true, 4, 0, &r->drv_ctrl2) && (r->drv_ctrl2 & 0x7eu) == CTRL2_TIMING;
        }
        else
        {
            ok = drv_reg(true, 7, 0, &r->drv_ctrl5) && (r->drv_ctrl5 & 3u) == 1u;
        }
        if(ok)
        {
            failed &= ~(1u << index);
        }
        else
        {
            failed |= 1u << index;
        }
    }
    slot = (slot + 1u) % 100u;
    r->enc_angle_ok = enc_read(0x3fffu, &s->encoder);
    if(!r->enc_angle_ok || !r->enc_diag_ok || (r->enc_diag & 0xf00u) != 0x100u)
    {
        s->errors |= ENCODER;
        if(diagnostics)
        {
            uint16_t ignored;
            enc_read(1, &ignored);
        }
    }
    if(failed)
    {
        s->errors |= CONFIG;
    }
    bool fault_h = (GPIO_ISTAT(GPIOA) & GPIO_PIN_6) != 0;
    if(fault_h || (r->drv_status[0] & 1u))
    {
        s->errors |= DRIVER;
    }
#ifndef AXIS_WAVE_OUTPUT
    r->fault_h = fault_h;
    if(!board_drv_detail_ok(r))
        s->errors |= DRIVER;
#endif
    if(r->mid < 1750 || r->mid > 2350)
    {
        s->errors |= SAMPLING;
    }
}

#ifndef AXIS_WAVE_OUTPUT
/* 顺序读取三个状态寄存器。高阻探测和关桥后的首次复读共用此路径。 */
bool board_read_drv_status(uint16_t status[3])
{
    return drv_reg(true, 0, 0, &status[0]) && drv_reg(true, 1, 0, &status[1]) &&
        drv_reg(true, 2, 0, &status[2]);
}

/* 主循环在空闲态独占 ADC/SPI。核对引脚实际电平、刹车使能及极性，保持主输出和自动重启关闭。 */
static bool hiz_guard(void)
{
    uint32_t mask = TIMER_CCHP_POEN | TIMER_CCHP_OAEN | TIMER_CCHP_BRKEN | TIMER_CCHP_BRKP;
    return (TIMER_CCHP(TIMER0) & mask) == (TIMER_CCHP_BRKEN | TIMER_CCHP_BRKP) &&
        !(GPIO_ISTAT(GPIOA) & (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) &&
        !(GPIO_ISTAT(GPIOB) & (GPIO_PIN_0 | GPIO_PIN_1)) && (GPIO_ISTAT(GPIOB) & NSLEEP);
}

bool board_is_off(void)
{
    return (GPIO_ISTAT(GPIOB) & DRVOFF) != 0;
}

/* 只在从关桥进入已确认的高阻态时清一次旧刹车标志。之后即使 TP8 回落也保留新刹车事件。 */
bool board_hiz_arm_break(void)
{
    if(board_is_off() || !hiz_guard() || (GPIO_ISTAT(GPIOA) & GPIO_PIN_6))
        return false;
    timer_flag_clear(TIMER0, TIMER_FLAG_BRK);
    return !(GPIO_ISTAT(GPIOA) & GPIO_PIN_6) && !(TIMER_INTF(TIMER0) & TIMER_FLAG_BRK);
}

/* 正常结束对齐或点动时撤销主输出。等一个死区后核实六个端口确实回低。 */
bool board_hiz_idle(void)
{
    if(running)
    {
        if(!(TIMER_CCHP(TIMER0) & TIMER_CCHP_POEN))
            return false;
        TIMER_CCHP(TIMER0) &= ~TIMER_CCHP_POEN;
        TIMER_CH0CV(TIMER0) = TIMER_CH1CV(TIMER0) = TIMER_CH2CV(TIMER0) = TOP / 2u;
        running = false;
        delay_cycles(180);
    }
    return !board_is_off() && hiz_guard() && !(GPIO_ISTAT(GPIOA) & GPIO_PIN_6) &&
        !(TIMER_INTF(TIMER0) & TIMER_FLAG_BRK);
}

/* 普通 6PWM 下每相两个输入同时为低即高阻。只改变 DRVOFF，定时器主输出始终关闭。 */
bool board_hiz_release(void)
{
    if(!hiz_guard() || !(GPIO_ISTAT(GPIOB) & DRVOFF))
        return false;
    GPIO_BC(GPIOB) = DRVOFF;
    return true;
}

/* 每个阶段等十组新的同步转换（约 1 ms）。若引脚状态变化或转换停摆，立即回到关桥。 */
void board_hiz_sample(hiz_reading_t* r)
{
    uint32_t start = DWT->CYCCNT;
    r->valid = false;
    r->off = (GPIO_ISTAT(GPIOB) & DRVOFF) != 0;
    r->fault_h = (GPIO_ISTAT(GPIOA) & GPIO_PIN_6) != 0;
    for(unsigned n = 0; n < 10; ++n)
    {
        adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
        do
        {
            r->guard = hiz_guard();
            if(!r->guard || DWT->CYCCNT - start >= 2u * TIMER_HZ / 1000u)
            {
                bridge_off();
                return;
            }
        } while(!adc_flag_get(ADC0, ADC_FLAG_EOIC));
    }
    for(unsigned i = 0; i < 4; ++i)
        r->raw.adc[i] = adc_inserted_data_read(ADC0, i);
    r->raw.mid = adc_routine_data_read(ADC1);
    r->valid = board_read_drv_status(r->raw.drv_status) && drv_reg(true, 4, 0, &r->raw.drv_ctrl2) &&
        drv_reg(true, 7, 0, &r->raw.drv_ctrl5);
    r->off = (GPIO_ISTAT(GPIOB) & DRVOFF) != 0;
    r->fault_h = (GPIO_ISTAT(GPIOA) & GPIO_PIN_6) != 0;
    r->guard = hiz_guard();
}
#endif

/* USART0_TX 使用 DMA0 通道 3，调用方保持缓冲区有效直至 DMA 读完。 */
static void uart_tx_start(const void* data, unsigned length)
{
    DMA_CHCTL(DMA0, DMA_CH3) = 0;
    dma_flag_clear(DMA0, DMA_CH3, DMA_FLAG_G);
    DMA_CHMADDR(DMA0, DMA_CH3) = (uint32_t)data;
    DMA_CHCNT(DMA0, DMA_CH3) = length;
    __DMB();
    DMA_CHCTL(DMA0, DMA_CH3) = DMA_CHXCTL_DIR | DMA_CHXCTL_MNAGA | DMA_CHXCTL_CHEN;
}

#ifdef AXIS_WAVE_OUTPUT
bool wave_send(const float* channels)
{
    static wave_frame_t frame;
    if(DMA_CHCNT(DMA0, DMA_CH3))
        return false;
    wave_pack(&frame, channels);
    uart_tx_start(&frame, sizeof(frame));
    return true;
}
#else
/* 仅主循环调用。等待本行传输时 ADC 中断仍可抢占，超时则停 DMA 后才释放缓冲区。 */
void uart_text(const char* line)
{
    uart_tx_start(line, (unsigned)strlen(line));
    uint32_t start = DWT->CYCCNT;
    while(DMA_CHCNT(DMA0, DMA_CH3))
    {
        if(DWT->CYCCNT - start >= TIMER_HZ / 100u)
        {
            DMA_CHCTL(DMA0, DMA_CH3) = 0;
            break;
        }
    }
}
#endif

/* -1 表示没有字节，-2 表示接收错误，调用者丢弃当前整行。 */
int uart_receive(void)
{
    static unsigned tail;
    /* 用 DMA 回卷标志区分空环和写指针追上读指针的情况。 */
    static unsigned wraps;
    if(dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF))
    {
        dma_flag_clear(DMA0, DMA_CH4, DMA_FLAG_FTF);
        wraps += 256;
    }
    /* wraps 累计已观察到的整圈字节数。head 是 DMA 累计写入位置，tail 是软件下一次读取位置。 */
    unsigned head = wraps + ((256u - DMA_CHCNT(DMA0, DMA_CH4)) & 255u);
    if(dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF))
    {
        return -1;
    }
    const uint32_t rx_errors = USART_STAT0_ORERR | USART_STAT0_FERR | USART_STAT0_NERR | USART_STAT0_PERR;
    if(head - tail >= 256 || (USART_STAT0(USART0) & rx_errors))
    {
        (void)USART_DATA(USART0);
        tail = head;
        return -2;
    }
    if(head == tail)
    {
        return -1;
    }
    return rx_ring[tail++ & 255u];
}
