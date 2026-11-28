/* R2.4 单轴驱动板外设接口。raw_t 保存硬件读数，sample_t 则携带控制器使用的物理量和角度码。 */
#ifndef AXIS_BOARD_H
#define AXIS_BOARD_H
#include "foc.h"

/* 单轴台架默认输出串口文本。联机轴沿用原波形，台架也可显式恢复原 VOFA+ 配置。 */
#ifdef AXIS_BENCH_VOFA
#define AXIS_WAVE_OUTPUT
#endif

typedef struct
{
    uint16_t adc[4]; /* ADC0 本拍的 12 位原始码，依次为 IA、IB、IC、母线分压。CH29～32。 */
    uint16_t mid; /* ADC1 读取的 DRV_AVDD_HALF 原始码，是三相零偏的核对基准。CH33。 */
    uint16_t enc_diag; /* AS5048A 最近一次成功读取的磁场诊断字。CH10。 */
    bool enc_diag_ok, enc_angle_ok; /* 各自最近一次 SPI 读数的有效性，失败时旧数值不可用于验板。 */
#ifndef AXIS_WAVE_OUTPUT
    bool fault_h; /* 本拍检查 DRIVER 时在 PA6 读到的 TP8 电平。 */
#endif
    uint16_t drv_status[3]; /* DRV8316C 最近读回的 IC_STAT、STAT1、STAT2。CH34～36。 */
    uint16_t drv_ctrl2; /* PWM 模式等配置的读回值。CH37。 */
    uint16_t drv_ctrl5; /* 电流检测增益等配置的读回值。CH38。 */
} raw_t;

static inline bool board_currents_at_mid(const raw_t* r)
{
    for(unsigned i = 0; i < 3; ++i)
    {
        int delta = (int)r->adc[i] - (int)r->mid;
        if(delta <= -70 || delta >= 70)
            return false;
    }
    return true;
}

/* 保护或任一相超窗都打断连续性，累计值在第 256 组完成前不能用于闭环。 */
static inline bool board_zero_sample(const raw_t* r, bool eligible, unsigned sum[3], unsigned* count)
{
    if(!eligible || !board_currents_at_mid(r))
    {
        sum[0] = sum[1] = sum[2] = *count = 0;
        return false;
    }
    for(unsigned i = 0; i < 3; ++i)
        sum[i] += r->adc[i];
    return ++*count == 256;
}

bool board_init(raw_t* raw);
void board_start_sampling(void);
void board_sample(raw_t* raw, sample_t* s, bool diagnostics);
bool bridge_set(bool enable, const float duty[3]);
void bridge_off(void);
#ifdef AXIS_WAVE_OUTPUT
bool wave_send(const float* channels);
#else
typedef struct
{
    const char* site; /* 首次拒绝或关桥的代码分支，复位前一直保留。 */
    bool running, tp8, off, pwm, brk;
    uint16_t port_a, port_b, timer_control;
} bridge_trip_t;
const bridge_trip_t* board_note_trip(const char* site);
bool board_read_drv_status(uint16_t status[3]);
typedef struct
{
    raw_t raw;
    bool valid, off, fault_h, guard; /* 完整新读数、PB11、TP8，以及六输入低且定时器安全关断。 */
} hiz_reading_t;
void board_hiz_sample(hiz_reading_t* reading);
bool board_hiz_release(void);
bool board_hiz_idle(void);
bool board_hiz_arm_break(void);
bool board_is_off(void);
/* DRV8316C 表 8-13 至 8-15。NPOR=1 表示供电正常，STAT2 位 7 为保留位。 */
static inline bool board_drv_detail_ok(const raw_t* r)
{
    return (r->drv_status[0] & 0x7eu) == 0x08u && !r->drv_status[1] && !(r->drv_status[2] & 0x7fu);
}
void uart_text(const char* line);
#endif
int uart_receive(void);
#endif
