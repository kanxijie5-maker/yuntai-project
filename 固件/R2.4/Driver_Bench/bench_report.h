/* 单轴台架串口自检。主循环使用同一个控制周期的快照生成整份报告。 */
#ifndef AXIS_BENCH_REPORT_H
#define AXIS_BENCH_REPORT_H
#include "board.h"

enum motor_check
{
    MOTOR_UNTESTED,
    MOTOR_TESTING,
    MOTOR_PASSED,
    MOTOR_FAILED,
    MOTOR_STOPPED
};

#ifndef AXIS_WAVE_OUTPUT
typedef struct
{
    bridge_trip_t hw;
    raw_t sample; /* 触发拍的 ADC 和周期 SPI 缓存，后续 status 不会覆盖。 */
    unsigned elapsed; /* 当前运动已执行的 100 µs 控制拍数。 */
    align_trace_t alignment; /* 回程故障时冻结三次位移，关桥后手动转轴也不影响报告。 */
    uint16_t post[3]; /* 关桥后首次主动复读的 IC_STAT、STAT1、STAT2。 */
    bool post_ok;
} bench_trip_t;
#endif

typedef struct
{
    raw_t raw;
    sample_t sample;
    unsigned ticks, live, faults;
    bool configured, calibrated, pwm, off;
    bool rpm_active;
    float cog_ff, speed_pi; /* 前馈和速度 PI 各自给出的 Iq，单位 A。 */
    bool cog_ready, cog_enabled;
    unsigned lead, eoc_min, eoc_max, tracking_n, commit_peak;
    float tracking_sq; /* 10 kHz 原始 Iq 跟踪误差平方和，单位 A²。 */
    float speed_ref, speed, iq_ref, iq; /* 同一拍的目标/实测机械速度(rad/s)和转矩电流(A)。 */
    enum motor_check motor; /* 最近一次手动带电测试的结果，关桥时不能重新判断绕组是否接入。 */
    enum mode mode;
#ifndef AXIS_WAVE_OUTPUT
    const bench_trip_t* trip;
#endif
} bench_sample_t;

enum motor_check bench_motor_update(enum motor_check previous, enum mode mode, bool aligned);
void bench_report(const bench_sample_t* s);
#ifndef AXIS_WAVE_OUTPUT
bool bench_hiz_prepare(hiz_reading_t* reading);
unsigned bench_hiz_probe(hiz_reading_t rows[3]);
void bench_hiz_report(const hiz_reading_t rows[3], unsigned count);
#endif
#endif
