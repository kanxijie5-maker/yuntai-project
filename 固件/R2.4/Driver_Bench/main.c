/* 固件入口。主循环接收上位机命令，ADC 中断按 PWM 节拍采样、运行 FOC 并记录波形。 */
#include "board.h"
#include "gd32f30x.h"
#include "wave.h"
#ifndef AXIS_WAVE_OUTPUT
#include "bench_report.h"
#endif
#include <math.h>
#include <string.h>

static axis_t axis;
static sample_t sample;
static raw_t raw;
static unsigned zeros[3]; /* 各相用于标定的 ADC 码累加和，除以 256 得零电流偏置。 */
static unsigned calibrated; /* 三相零偏全部就绪时为 1，对应 CH4。 */
static unsigned isr_cycles; /* 进入上一次采样中断至遥测处理结束的 CPU 周期数，CH43 换算为 µs。 */
static unsigned live; /* 当前异常位，按本拍观测更新。锁存的停机原因保存在 axis.faults。 */
static unsigned commands; /* 已处理的控制命令及无效命令行累计数，对应 CH7。 */
static bool configured; /* 初始化时已成功写入并读回 DRV8316C 的 PWM 模式与电流检测增益。 */
static unsigned calib_count; /* 当前连续合格的零偏样本组数，中途异常时重新累计。 */
static uint32_t last_adc_cycle; /* 用于检查相邻采样中断的间隔。 */
/* 主循环发布请求，ADC 中断消费。pending 置位表示 mailbox 中已有一条完整命令。 */
static volatile bool pending, bad_command;
static volatile command_t mailbox;
static volatile bool scope_timing; /* 串口 timing 将 PA5 心跳切为采样中断入口至 PWM 提交的脉冲。 */

/* 正常以 250 Hz 发送。触发录波时按控制频率缓存 128 行，再由 DMA 按串口带宽发出。 */
#ifdef AXIS_WAVE_OUTPUT
static unsigned dropped; /* DMA 忙而跳过的常规遥测帧累计数，对应 CH44。 */
static volatile bool capture_request;

enum
{
    CAPACITY = 128
};

static float capture[CAPACITY][CHANNELS];
static unsigned captured, transmitted; /* 已存入的快照数、已交给 DMA 的回放帧数。 */
/* capture 命令使 armed_capture 等待下一条运动给定，再依次进入 recording 采样和 draining 回放。 */
static bool recording, draining, armed_capture;

/* 按通道表生成同一控制周期的快照，录波回放也使用本次采样的时间戳。 */
static void pack(float* f)
{
    const axis_t* a = &axis;
    float values[CHANNELS] = {(float)(a->tick % 10000000u) / CONTROL_HZ, a->mode, a->faults, live, calibrated,
        a->aligned, a->command_result, commands, sample.bus, sample.encoder, raw.enc_diag, a->direction,
        a->position, a->speed, a->angle,
        /* CH15～23 从三相反馈开始，依次观察 Id 给定/反馈、Iq 给定/反馈，以及两轴输出电压。 */
        sample.phase[0], sample.phase[1], sample.phase[2], 0, a->id, a->iq_ref, a->iq, a->vd, a->vq,
        a->mode == POSITION ? a->target : 0, a->speed_ref, a->duty[0], a->duty[1], a->duty[2], raw.adc[0],
        raw.adc[1], raw.adc[2], raw.adc[3], raw.mid, raw.drv_status[0], raw.drv_status[1], raw.drv_status[2],
        raw.drv_ctrl2, raw.drv_ctrl5, a->pid[D_LOOP].integral, a->pid[Q_LOOP].integral,
        a->pid[SPEED_LOOP].integral, a->pid[POS_LOOP].integral, isr_cycles / 120.0f, dropped,
        armed_capture || recording || draining, a->lease / 10.0f, a->target,
        /* CH48～59 按 d、q、速度、位置顺序回显每个环实际使用的 Kp、Ki、Kd。 */
        a->pid[D_LOOP].kp, a->pid[D_LOOP].ki, a->pid[D_LOOP].kd, a->pid[Q_LOOP].kp, a->pid[Q_LOOP].ki,
        a->pid[Q_LOOP].kd, a->pid[SPEED_LOOP].kp, a->pid[SPEED_LOOP].ki, a->pid[SPEED_LOOP].kd,
        a->pid[POS_LOOP].kp, a->pid[POS_LOOP].ki, a->pid[POS_LOOP].kd, zeros[0] / 256.0f, zeros[1] / 256.0f,
        zeros[2] / 256.0f, (float)(TIMER_CCHP(TIMER0) & TIMER_CCHP_POEN ? 1 : 0)};
    memcpy(f, values, sizeof(values));
}
#else
static volatile enum motor_check motor_result;
static bench_trip_t first_trip; /* ADC 中断只写首个故障，主循环在关桥后打印。 */
static volatile bool hiz_armed; /* 台架已通过六输入低电平检查，允许在 PWM 关闭时保持采样电路工作。 */
static unsigned cal_start_tick;
static unsigned eoc_min, eoc_max, tracking_n; /* 稳态转速期间 ADC 组完成中断入口的计数器范围。 */
static unsigned commit_peak; /* rpm 运行期间从中断入口到提交前检查的最坏 CPU 周期数。 */
static float tracking_sq;

/* 临界区只复制采样与保护状态。格式化串口文本前恢复中断。 */
static void report_status(void)
{
    bench_sample_t s;
    __disable_irq();
    s.raw = raw;
    s.sample = sample;
    s.ticks = axis.tick;
    s.mode = axis.mode;
    s.rpm_active = axis.hold_rpm;
    s.cog_ready = axis_cog_ready(&axis);
    s.cog_enabled = axis.cog_enabled && s.cog_ready;
    s.cog_ff = axis.cog_current;
    const pid_tune_t* p = &axis.pid[SPEED_LOOP];
    s.speed_pi =
        axis.hold_rpm ? p->kp * (axis.speed_ref - p->previous) + p->integral - p->kd * p->derivative : 0;
    s.lead = TOP - TIMER_CH3CV(TIMER0);
    s.eoc_min = eoc_min;
    s.eoc_max = eoc_max;
    s.tracking_n = tracking_n;
    s.tracking_sq = tracking_sq;
    s.commit_peak = commit_peak;
    s.speed_ref = axis.target;
    s.speed = axis.speed;
    s.iq_ref = axis.iq_ref;
    s.iq = axis.iq;
    s.live = live;
    s.faults = axis.faults;
    s.configured = configured;
    s.calibrated = calibrated != 0;
    s.pwm = (TIMER_CCHP(TIMER0) & TIMER_CCHP_POEN) != 0;
    s.off = board_is_off();
    s.motor = motor_result;
    s.trip = first_trip.hw.site ? &first_trip : NULL;
    __enable_irq();
    bench_report(&s);
}

/* 前台独占采样结束，丢弃窗口中最后的中断请求，从下一组新转换重新检查节拍。 */
static void resume_sampling(void)
{
    last_adc_cycle = 0;
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
    NVIC_ClearPendingIRQ(ADC0_1_IRQn);
    NVIC_EnableIRQ(ADC0_1_IRQn);
}

/* 仅从空闲进入高阻待机。校准使用中断后续采集的 256 组，单次探测值不能充当零偏。 */
static bool prepare_idle(bool recalibrate)
{
    hiz_reading_t s = {0};
    NVIC_DisableIRQ(ADC0_1_IRQn);
    __DSB();
    __ISB();
    if(axis.mode != IDLE || axis.faults || pending || !configured || axis.tick < 100 ||
        (live & (SUPPLY | CONFIG | DEADLINE)) || (TIMER_CCHP(TIMER0) & TIMER_CCHP_POEN))
    {
        NVIC_EnableIRQ(ADC0_1_IRQn);
        return false;
    }
    bridge_off();
    hiz_armed = bench_hiz_prepare(&s);
    if(hiz_armed)
    {
        /* 替换关桥时缓存的故障字，后续仍由周期 SPI 复查。 */
        memcpy(raw.drv_status, s.raw.drv_status, sizeof(raw.drv_status));
        cal_start_tick = axis.tick;
    }
    else if(!board_is_off())
        axis_trip(&axis, DRIVER);
    if(recalibrate || !hiz_armed || !calibrated)
    {
        calibrated = calib_count = 0;
        memset(zeros, 0, sizeof(zeros));
    }
    bool ok = hiz_armed;
    resume_sampling();
    return ok;
}

/* 空闲态暂由主循环轮询 ADC 转换完成标志，硬件触发、BKIN 和看门狗一直工作。 */
static void report_hiz(void)
{
    hiz_reading_t rows[3] = {0};
    NVIC_DisableIRQ(ADC0_1_IRQn);
    __DSB();
    __ISB();
    if(axis.mode != IDLE || axis.faults || pending || !configured || axis.tick < 100 ||
        (live & ~(DRIVER | OVERCURRENT | SAMPLING)) || (TIMER_CCHP(TIMER0) & TIMER_CCHP_POEN))
    {
        NVIC_EnableIRQ(ADC0_1_IRQn);
        uart_text("PROBE REJECT 请用 status 核对采样节拍、配置、空闲状态及锁存故障\r\n");
        return;
    }
    hiz_armed = false;
    bridge_off();
    unsigned count = bench_hiz_probe(rows);
    if(!rows[count - 1].off)
        axis_trip(&axis, DRIVER);
    /* 对照期间的样本只用于报告。恢复时从下一组转换重新计时和累计未完成的连续零偏样本。 */
    if(!calibrated)
    {
        memset(zeros, 0, sizeof(zeros));
        calib_count = 0;
    }
    resume_sampling();
    bench_hiz_report(rows, count);
}
#endif

/* 零电流时以 PA4 为交叉参考，连续 256 组 ADC 都合格才采用三相各自的偏置。 */
static void read_currents(void)
{
#ifndef AXIS_WAVE_OUTPUT
    /* DRVOFF 关断时三个 SO 输出会贴近电源轨，原码照常报告，禁止把它们换算成相电流。 */
    if(!hiz_armed || board_is_off())
    {
        memset(sample.phase, 0, sizeof(sample.phase));
        return;
    }
#endif
    if(!calibrated)
    {
        bool good = configured && !(sample.errors & (CONFIG | DRIVER | SAMPLING)) && sample.bus >= 8 &&
            sample.bus <= 15;
        calibrated = board_zero_sample(&raw, good, zeros, &calib_count);
#ifndef AXIS_WAVE_OUTPUT
        if(!calibrated)
        {
            memset(sample.phase, 0, sizeof(sample.phase));
            return;
        }
#endif
    }
    for(unsigned i = 0; i < 3; ++i)
    {
        float offset = calibrated ? zeros[i] / 256.0f : raw.mid;
        /* 扣除偏置后的码差乘 3.3 V / 4095 得到电压，再除以驱动芯片的 0.30 V/A 得到相电流。 */
        sample.phase[i] = (raw.adc[i] - offset) * (3.3f / (4095.0f * 0.30f));
        if(raw.adc[i] < 10 || raw.adc[i] > 4085 || (calibrated && fabsf(offset - raw.mid) > 100))
        {
            sample.errors |= SAMPLING;
        }
    }
    /* 三线电机的三相电流之和应接近零。残差超过 0.16 A 时，标记这组采样异常。 */
    if(fabsf(sample.phase[0] + sample.phase[1] + sample.phase[2]) > 0.16f)
    {
        sample.errors |= SAMPLING;
    }
}

/* 实时模式按 40 分频发送。录波先存 RAM，再等 DMA 空闲逐帧回放。 */
#ifdef AXIS_WAVE_OUTPUT
static void telemetry(void)
{
    if(recording)
    {
        pack(capture[captured++]);
        if(captured == CAPACITY)
        {
            recording = false;
            draining = true;
            transmitted = 0;
        }
    }
    else if(draining)
    {
        if(wave_send(capture[transmitted]) && ++transmitted == CAPACITY)
        {
            draining = false;
        }
    }
    else if((axis.tick % 40u) == 0)
    {
        float frame[CHANNELS];
        pack(frame);
        if(!wave_send(frame))
        {
            ++dropped;
        }
    }
}
#endif

/* 唯一修改电机状态的上下文。先采样和检查保护，再处理命令、运行闭环和更新三相占空比。 */
void ADC0_1_IRQHandler(void)
{
    uint32_t now = DWT->CYCCNT;
    if(scope_timing)
        GPIO_BOP(GPIOA) = GPIO_PIN_5;
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
    sample.errors = 0;
    /* 第四路转换结束约在顶点后 7.23 µs，检查载波相位可发现错半拍或中断迟到。 */
    uint32_t count = TIMER_CNT(TIMER0);
    if(TIMER_CH3CV(TIMER0) != TOP - TRIGGER_LEAD || !(TIMER_CTL0(TIMER0) & TIMER_CTL0_DIR) ||
        TIMER_CNT(TIMER1) != 0 || count < 1550u || count > 2400u)
        sample.errors |= DEADLINE;
    /* 120 MHz 下每周期应有 12000 个时钟。也在空闲态检测重复触发和漏触发。 */
    if(last_adc_cycle && (now - last_adc_cycle < 10500u || now - last_adc_cycle > 13500u))
    {
        sample.errors |= DEADLINE;
    }
    last_adc_cycle = now;
    board_sample(&raw, &sample, (axis.tick % 100u) == 0);
#ifndef AXIS_WAVE_OUTPUT
    if(hiz_armed && board_is_off())
        sample.errors |= DRIVER;
#endif
    read_currents();
    live = sample.errors;
    if(sample.bus < 8 || sample.bus > 15)
    {
        live |= SUPPLY;
    }
    for(unsigned i = 0; i < 3; ++i)
    {
        if(fabsf(sample.phase[i]) >= 0.65f)
        {
            live |= OVERCURRENT;
        }
    }
    bool invalidate_alignment = live && axis.mode == IDLE;
#ifndef AXIS_WAVE_OUTPUT
    if(!hiz_armed && board_is_off() && live == DRIVER && board_drv_detail_ok(&raw))
        invalidate_alignment = false;
#endif
    if(invalidate_alignment)
        axis.aligned = false;
#ifndef AXIS_WAVE_OUTPUT
    if(hiz_armed && axis.mode == IDLE)
    {
        unsigned trips = live & ~ENCODER;
        if(!calibrated && axis.tick - cal_start_tick >= CONTROL_HZ / 10u)
            trips |= SAMPLING;
        if(trips)
            axis_trip(&axis, trips);
    }
    if(live & (SUPPLY | CONFIG))
    {
        calibrated = calib_count = 0;
        memset(zeros, 0, sizeof(zeros));
    }
#endif
    if(bad_command)
    {
        bad_command = false;
        axis.command_result = 4;
        ++commands;
    }
#ifdef AXIS_WAVE_OUTPUT
    if(capture_request)
    {
        capture_request = false;
        armed_capture = !draining && !recording;
    }
#endif
    if(pending)
    {
        command_t c = mailbox;
        pending = false;
        ++commands;
#ifndef AXIS_WAVE_OUTPUT
        if(c.op == STOP)
        {
            hiz_armed = false;
            bridge_off();
        }
        axis_command(&axis, &c, hiz_armed && configured && calibrated && !live && axis.encoder_seen);
#else
        axis_command(&axis, &c, configured && calibrated && !live && axis.encoder_seen);
#endif

#ifdef AXIS_WAVE_OUTPUT
        if(armed_capture && axis.command_result == 1 && c.op >= START_ALIGN && c.op <= SET_POS)
        {
            armed_capture = false;
            captured = 0;
            recording = true;
        }
#else
        if(c.op == START_ALIGN && axis.command_result == 1)
            motor_result = MOTOR_TESTING;
#endif
    }
    /* axis_step 决定本拍的状态和占空比，随后 bridge_set 将结果送到定时器并检查硬件刹车。 */
#ifndef AXIS_WAVE_OUTPUT
    float prior_iq_ref = axis.iq_ref;
#endif
    axis_step(&axis, &sample);
#ifndef AXIS_WAVE_OUTPUT
    if(axis.hold_rpm && axis.elapsed == 1u)
    {
        tracking_n = eoc_max = 0;
        commit_peak = 0;
        eoc_min = TOP;
        tracking_sq = 0;
    }
    if(axis.hold_rpm && axis.elapsed > 5u * CONTROL_HZ && tracking_n < 90u * CONTROL_HZ)
    {
        float error = axis.iq - prior_iq_ref;
        /* 使用未滤波的本拍 Iq 和上一拍给定，保留运行时电流跟踪误差统计。 */
        tracking_sq += error * error;
        ++tracking_n;
        if(count < eoc_min)
            eoc_min = count;
        if(count > eoc_max)
            eoc_max = count;
    }
#endif
    bool enable = axis.mode > IDLE && axis.mode < FAULT;
#ifndef AXIS_WAVE_OUTPUT
    if(axis.mode == FAULT && hiz_armed)
        board_note_trip((axis.faults & DRIVER) ? "SAMPLE" : "CONTROL");
#endif
    unsigned commit_cycles = DWT->CYCCNT - now;
#ifndef AXIS_WAVE_OUTPUT
    if(axis.hold_rpm && commit_cycles > commit_peak)
        commit_peak = commit_cycles;
#endif
    if(enable && commit_cycles >= COMMIT_BUDGET)
    {
#ifndef AXIS_WAVE_OUTPUT
        board_note_trip("COMMIT_TIME");
#endif
        bridge_off();
        axis_trip(&axis, DEADLINE);
    }
#ifndef AXIS_WAVE_OUTPUT
    else if(axis.mode == IDLE && hiz_armed)
    {
        if(!board_hiz_idle())
        {
            board_note_trip("IDLE_HIZ");
            bridge_off();
            axis_trip(&axis, DRIVER);
        }
    }
#endif
    else if(!bridge_set(enable, axis.duty))
    {
        axis_trip(&axis, DRIVER);
    }
    gpio_bit_write(GPIOA, GPIO_PIN_12, axis.mode == FAULT || live ? SET : RESET);
    gpio_bit_write(GPIOA, GPIO_PIN_5, !scope_timing && axis.tick % CONTROL_HZ < CONTROL_HZ / 2 ? SET : RESET);
#ifdef AXIS_WAVE_OUTPUT
    telemetry();
#endif
    isr_cycles = DWT->CYCCNT - now;
    if(enable && isr_cycles >= 11500u)
    {
#ifndef AXIS_WAVE_OUTPUT
        board_note_trip("ISR_TIME");
#endif
        bridge_off();
        axis_trip(&axis, DEADLINE);
    }
#ifndef AXIS_WAVE_OUTPUT
    if(axis.mode == FAULT)
    {
        if(hiz_armed && !first_trip.hw.site)
        {
            /* 保护分支已先关桥。此处才保存采样和复读 SPI，串口不会阻塞安全动作。 */
            first_trip.hw = *board_note_trip("SHUTDOWN");
            first_trip.elapsed = axis.elapsed;
            first_trip.alignment = axis.align_trace;
            first_trip.sample = raw;
            first_trip.post_ok = board_read_drv_status(first_trip.post);
        }
        hiz_armed = false;
        calibrated = 0;
    }
    if(motor_result == MOTOR_TESTING)
        motor_result = bench_motor_update(motor_result, axis.mode, axis.aligned);
#endif
    /* 看门狗在采样中断末尾喂，采样中断停摆时主循环无法代为续期。 */
    FWDGT_CTL = FWDGT_KEY_RELOAD;
}

/* 主循环只拼接命令行；中断关闭区只做一次信箱复制，浮点解析与串口传输均不阻塞电流环。 */
int main(void)
{
    axis_init(&axis);
    configured = board_init(&raw);
    if(!configured)
    {
        axis_trip(&axis, CONFIG);
    }
#ifndef AXIS_WAVE_OUTPUT
    uart_text("\r\n[R2.4 GD32F303 驱动板自检] 921600 8N1 UTF-8\r\n"
              "上电高阻零偏校准，POEN 回读窗口 1 us。status 查状态及 TRIP 首次故障，cal 重校，probe "
              "查高阻采样，motor 带电测试，rpm10 单发持续调速，stop 关桥。\r\n");
    if(RCU_RSTSCK & RCU_RSTSCK_FWDGTRSTF)
        uart_text("WATCHDOG 上次看门狗复位，请检查采样中断和 ADC 校准\r\n");
    RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
#endif
    rcu_osci_on(RCU_IRC40K);
    fwdgt_config(500, FWDGT_PSC_DIV4);
    fwdgt_counter_reload();
    fwdgt_enable();
    board_start_sampling();
    char line[80];
    unsigned len = 0;
    bool discard = false;
#ifndef AXIS_WAVE_OUTPUT
    /* 首报等 100 ms 让零偏校准完成；之后即使只接了板 TX 也能每 2 秒看到结果。 */
    uint32_t last_report = DWT->CYCCNT - (2u * TIMER_HZ - TIMER_HZ / 10u);
    enum motor_check last_motor = MOTOR_UNTESTED;
    bool acknowledge = false;
    bool boot_cal = true;
#endif
    for(;;)
    {
        if(pending || bad_command)
        {
            continue;
        }
#ifdef AXIS_WAVE_OUTPUT
        if(capture_request)
            continue;
#else
        if(boot_cal && axis.tick >= 100)
        {
            boot_cal = false;
            uart_text(prepare_idle(true) ? "CAL HIZ 正在采集 256 组零偏\r\n"
                                         : "CAL REJECT 高阻准备失败，请查 status 和 probe\r\n");
        }
        if(acknowledge)
        {
            __DMB();
            static const char* const replies[] = {"", "CMD OK\r\n",
                "CMD REJECT 硬件未就绪或故障已锁存，发送 status 检查\r\n",
                "CMD REJECT 当前状态不允许，请先检查对齐或 stop\r\n", "CMD ERROR 指令无效\r\n"};
            uart_text(replies[axis.command_result]);
            acknowledge = false;
        }
        enum motor_check result = motor_result;
        if(result != last_motor || DWT->CYCCNT - last_report >= 2u * TIMER_HZ)
        {
            last_motor = result;
            last_report = DWT->CYCCNT;
            report_status();
        }
#endif
        int ch = uart_receive();
        if(ch == -1)
        {
            continue;
        }
        if(ch == '\n' || ch == '\r')
        {
            if(discard)
            {
                bad_command = true;
#ifndef AXIS_WAVE_OUTPUT
                acknowledge = true;
#endif
            }
            else if(len)
            {
                line[len] = 0;
                command_t c;
                if(!strcmp(line, "timing") || !strcmp(line, "heartbeat"))
                {
                    scope_timing = line[0] == 't';
                }
#ifdef AXIS_WAVE_OUTPUT
                else if(!strcmp(line, "capture"))
                {
                    capture_request = true;
                }
#else
                else if(!strcmp(line, "status"))
                {
                    report_status();
                    last_report = DWT->CYCCNT;
                }
                else if(!strcmp(line, "probe"))
                {
                    report_hiz();
                }
                else if(!strcmp(line, "cal"))
                {
                    boot_cal = false;
                    uart_text(prepare_idle(true) ? "CAL HIZ 正在采集 256 组零偏\r\n"
                                                 : "CAL REJECT 请检查供电、模式、刹车和状态寄存器\r\n");
                }
#endif
                else if(command_parse(line, &c))
                {
#ifndef AXIS_WAVE_OUTPUT
                    if(c.op == STOP)
                        boot_cal = false;
                    if((c.op >= START_ALIGN && c.op <= SET_POS) && !hiz_armed && axis.mode == IDLE)
                        prepare_idle(false);
                    acknowledge = c.op != KEEP;
#endif
                    __disable_irq();
                    mailbox = c;
                    pending = true;
                    __enable_irq();
                }
                else
                {
                    bad_command = true;
#ifndef AXIS_WAVE_OUTPUT
                    acknowledge = true;
#endif
                }
            }
            len = 0;
            discard = false;
        }
        else if(ch < 32 || ch > 126 || len == sizeof(line) - 1)
        {
            discard = true;
        }
        else if(!discard)
        {
            line[len++] = (char)ch;
        }
    }
}
