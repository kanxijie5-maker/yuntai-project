/* 时序统一用 TIMER0 的 120 MHz 计数表示，便于把示波器读数与固件核对。 */
#ifndef AXIS_TIMING_H
#define AXIS_TIMING_H
#define TIMER_HZ 120000000u
#define PWM_HZ 20000u
#define CONTROL_HZ 10000u
#define TOP (TIMER_HZ / (2u * PWM_HZ))
#define CONTROL_TICKS (TIMER_HZ / CONTROL_HZ)
#define TRIGGER_LEAD 300u /* 顶点前 2.50 µs 开始三相顺序采样。 */
#define COMMIT_BUDGET 6600u /* ADC 中断入口后 55 µs 内提交三路影子寄存器。 */
#define POEN_CONFIRM_TICKS (TIMER_HZ / 1000000u) /* 首次开启 PWM 的软件读回确认窗口，1 µs。 */
#define MAX_DUTY_PERCENT 65u
/* ADC 15 MHz，三相各 (13.5+12.5) 周期，第四路母线 (55.5+12.5) 周期。 */
#define PHASE_ADC_TICKS (3u * 26u * 8u)
#define GROUP_ADC_TICKS (PHASE_ADC_TICKS + 68u * 8u)
#define EDGE_CLEARANCE (TOP - TOP * MAX_DUTY_PERCENT / 100u)
_Static_assert(PWM_HZ == 2u * CONTROL_HZ && TOP == 3000u, "R2.4 timer divider");
_Static_assert(EDGE_CLEARANCE >= TRIGGER_LEAD + 600u, "allow 5 us settling before first aperture");
_Static_assert(EDGE_CLEARANCE + TRIGGER_LEAD >= PHASE_ADC_TICKS + 600u, "allow 5 us after phase group");
_Static_assert(TOP - TRIGGER_LEAD + GROUP_ADC_TICKS + COMMIT_BUDGET + 1000u < CONTROL_TICKS,
    "leave time for IRQ entry and shadow transfer");
#endif
