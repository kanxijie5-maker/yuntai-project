/* 普通串口助手使用 UTF-8。所有格式化和发送均在主循环内完成。 */
#include "bench_report.h"
#ifndef AXIS_WAVE_OUTPUT
#include <math.h>
#include <stdio.h>

enum motor_check bench_motor_update(enum motor_check previous, enum mode mode, bool aligned)
{
    if(previous != MOTOR_TESTING || mode == ALIGN)
        return previous;
    if(mode == FAULT)
        return MOTOR_FAILED;
    return mode == IDLE && aligned ? MOTOR_PASSED : MOTOR_STOPPED;
}

/* 允许检查已知的关桥总故障和失真的空闲电流，但具体驱动故障、配置和供电仍须逐项检查。 */
static bool hiz_valid(const hiz_reading_t* s)
{
    const raw_t* r = &s->raw;
    float bus = r->adc[3] * (3.3f * 11.0f / 4095.0f);
    return s->valid && s->guard && (r->drv_ctrl2 & 0x7eu) == 0x70u && (r->drv_ctrl5 & 3u) == 1u &&
        board_drv_detail_ok(r) && bus >= 8 && bus <= 15 && r->mid >= 1750 && r->mid <= 2350;
}

static bool hiz_start(hiz_reading_t* before)
{
    board_hiz_sample(before);
    return hiz_valid(before) && before->off && board_hiz_release();
}

/* 校准前先验证真实高阻状态。这里只允许 ADC 开始累计，PWM 仍由运动命令单独使能。 */
bool bench_hiz_prepare(hiz_reading_t* reading)
{
    if(hiz_start(reading))
    {
        board_hiz_sample(reading);
        if(hiz_valid(reading) && !reading->off && !reading->fault_h && !(reading->raw.drv_status[0] & 1u) &&
            board_currents_at_mid(&reading->raw) && board_hiz_arm_break())
            return true;
    }
    bridge_off();
    return false;
}

unsigned bench_hiz_probe(hiz_reading_t rows[3])
{
    if(!hiz_start(&rows[0]))
    {
        bridge_off();
        return 1;
    }
    board_hiz_sample(&rows[1]);
    bridge_off();
    board_hiz_sample(&rows[2]);
    return 3;
}

void bench_hiz_report(const hiz_reading_t rows[3], unsigned count)
{
    static const char* const stage[] = {"BEFORE", "HIZ", "RESTORED"};
    for(unsigned i = 0; i < count; ++i)
    {
        const hiz_reading_t* s = &rows[i];
        const raw_t* r = &s->raw;
        char line[200];
        snprintf(line, sizeof(line),
            "PROBE %s VALID=%u OFF=%u TP8=%u GUARD=%u ADC=%u/%u/%u MID=%u VM_RAW=%u "
            "STAT=%02X/%02X/%02X CTRL=%02X/%02X\r\n",
            stage[i], (unsigned)s->valid, (unsigned)s->off, (unsigned)s->fault_h, (unsigned)s->guard,
            (unsigned)r->adc[0], (unsigned)r->adc[1], (unsigned)r->adc[2], (unsigned)r->mid,
            (unsigned)r->adc[3], (unsigned)r->drv_status[0], (unsigned)r->drv_status[1],
            (unsigned)r->drv_status[2], (unsigned)r->drv_ctrl2, (unsigned)r->drv_ctrl5);
        uart_text(line);
    }
    if(count == 1)
        uart_text("PROBE REJECT 初始读数或关桥条件未通过\r\n");
    else if(!rows[2].off)
        uart_text("PROBE ERROR DRVOFF 未恢复高电平，请断电检查\r\n");
    else if(!rows[1].valid || !rows[1].guard || rows[1].off || !rows[2].valid || !rows[2].guard)
        uart_text("PROBE INVALID 对照条件或采样无效，已请求关桥\r\n");
}

void bench_report(const bench_sample_t* s)
{
    static const char* const motors[] = {"SKIP 尚未执行带电测试", "RUN 对齐测试中",
        "PASS 上次电流响应、转子扫动和回程通过", "FAIL 上次带电测试失败，请查故障位", "STOP 上次测试已中止"};
    if(!s->ticks)
    {
        uart_text("ADC WAIT 等待 PWM 同步采样\r\nMOTOR SKIP 尚无采样\r\n");
        return;
    }
    const raw_t* r = &s->raw;
    char line[240];
    const char* drv = s->configured && !(s->live & (CONFIG | DRIVER)) ? "PASS" : "FAIL";
    if(s->off && s->mode == IDLE && s->configured && !(s->live & CONFIG) && board_drv_detail_ok(r))
        drv = "OFF";
    snprintf(line, sizeof(line),
        "VM %s 母线=%u mV\r\nDRV8316CR %s DRVOFF=%u CTRL2/5=%02X/%02X STAT=%02X/%02X/%02X\r\n",
        s->live & SUPPLY ? "FAIL" : "PASS", (unsigned)(s->sample.bus * 1000 + 0.5f), drv, (unsigned)s->off,
        (unsigned)r->drv_ctrl2, (unsigned)r->drv_ctrl5, (unsigned)r->drv_status[0],
        (unsigned)r->drv_status[1], (unsigned)r->drv_status[2]);
    uart_text(line);
    const char* adc = s->live & (SAMPLING | OVERCURRENT) ? "FAIL" : s->calibrated ? "PASS" : "WAIT";
    if(s->off && !(s->live & (SAMPLING | OVERCURRENT)))
        adc = "OFF";
    snprintf(line, sizeof(line), "ADC %s 三相原码=%u/%u/%u 中点=%u 零偏就绪=%u\r\n", adc, (unsigned)r->adc[0],
        (unsigned)r->adc[1], (unsigned)r->adc[2], (unsigned)r->mid, (unsigned)s->calibrated);
    uart_text(line);
    if(!r->enc_angle_ok || !r->enc_diag_ok)
        uart_text("AS5048A FAIL SPI 帧无效，检查 J4 供电、接线及编码器\r\n");
    else
    {
        snprintf(line, sizeof(line), "AS5048A %s 角度码=%u DIAG=0x%03X\r\n",
            s->live & ENCODER ? "FAIL 检查磁铁、间隙及诊断位" : "PASS", (unsigned)s->sample.encoder,
            (unsigned)r->enc_diag);
        uart_text(line);
    }
    snprintf(line, sizeof(line), "TIMING %s 当前异常=0x%03X 锁存=0x%03X 模式=%u PWM=%s\r\nMOTOR %s\r\n",
        s->live & DEADLINE ? "FAIL" : "PASS", s->live, s->faults, (unsigned)s->mode, s->pwm ? "ON" : "OFF",
        motors[s->motor]);
    uart_text(line);
    if(s->rpm_active)
    {
        /* 用整数打印百分之一转/分，正反转和不足 1 rpm 的速度都能直接对照。 */
        snprintf(line, sizeof(line), "RPM 目标/反馈[0.01rpm]=%d/%d Iq目标/反馈[mA]=%d/%d\r\n",
            (int)roundf(s->speed_ref * (6000.0f / TAU_F)), (int)roundf(s->speed * (6000.0f / TAU_F)),
            (int)roundf(s->iq_ref * 1000.0f), (int)roundf(s->iq * 1000.0f));
        uart_text(line);
    }
    unsigned error_ua = s->tracking_n ? (unsigned)(1e6f * sqrtf(s->tracking_sq / s->tracking_n) + 0.5f) : 0;
    snprintf(line, sizeof(line), "TRIAL OPT=21 TICK=%u LEAD=%u EOC=%u/%u IQ_RMS_UA=%u N=%u PEAK_CLK=%u\r\n",
        s->ticks, s->lead, s->tracking_n ? s->eoc_min : 0, s->eoc_max, error_ua, s->tracking_n,
        s->commit_peak);
    uart_text(line);
    snprintf(line, sizeof(line), "FEED PCT=100 SHIFT=0 FF_UA=%d PI_UA=%d\r\n", (int)roundf(1e6f * s->cog_ff),
        (int)roundf(1e6f * s->speed_pi));
    uart_text(line);
    snprintf(line, sizeof(line), "COG TABLE=D_SMOOTH READY=%u ON=%u\r\n", (unsigned)s->cog_ready,
        (unsigned)s->cog_enabled);
    uart_text(line);
    if(s->faults && s->trip)
    {
        const bench_trip_t* t = s->trip;
        const bridge_trip_t* h = &t->hw;
        const raw_t* r0 = &t->sample;
        snprintf(line, sizeof(line),
            "TRIP SRC=%s STEP=%u RUN=%u TP8=%u OFF=%u POEN=%u BRK=%u PA=%04X PB=%04X CCHP=%04X\r\n", h->site,
            t->elapsed, (unsigned)h->running, (unsigned)h->tp8, (unsigned)h->off, (unsigned)h->pwm,
            (unsigned)h->brk, (unsigned)h->port_a, (unsigned)h->port_b, (unsigned)h->timer_control);
        uart_text(line);
        if((s->faults & ALIGNMENT) && t->elapsed == ALIGN_RETURN_STEP)
        {
            int limit = (int)(ALIGN_RETURN_LIMIT_RAD * 16384.0f / (TAU_F * POLE_PAIRS));
            snprintf(line, sizeof(line), "ALIGN RETURN SWEEP=%d PRE100=%d END=%d LIMIT=%d counts\r\n",
                t->alignment.sweep, t->alignment.before_end, t->alignment.end, limit);
            uart_text(line);
        }
        snprintf(line, sizeof(line),
            "TRIP SAMPLE_TP8=%u ADC=%u/%u/%u MID=%u VM_RAW=%u STAT_CACHE=%02X/%02X/%02X\r\n",
            (unsigned)r0->fault_h, (unsigned)r0->adc[0], (unsigned)r0->adc[1], (unsigned)r0->adc[2],
            (unsigned)r0->mid, (unsigned)r0->adc[3], (unsigned)r0->drv_status[0], (unsigned)r0->drv_status[1],
            (unsigned)r0->drv_status[2]);
        uart_text(line);
        if(t->post_ok)
        {
            snprintf(line, sizeof(line), "TRIP POST_STAT=%02X/%02X/%02X\r\n", (unsigned)t->post[0],
                (unsigned)t->post[1], (unsigned)t->post[2]);
            uart_text(line);
        }
        else
            uart_text("TRIP POST_STAT=SPI_ERROR\r\n");
    }
    else if(s->faults)
        uart_text("TRIP 暂无启动或运行时快照\r\n");
}
#endif
