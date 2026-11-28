/* 主机运行实际串口报告逻辑，验证缺件、失效读数和人为停止不会显示电机通过。 */
#include "../bench_report.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static char output[2048];
static hiz_reading_t observed[3];
static unsigned reads, releases, closes;
static bool release_ok, brake_ok;

void board_hiz_sample(hiz_reading_t* r)
{
    assert(reads < 3);
    *r = observed[reads++];
}

bool board_hiz_release(void)
{
    ++releases;
    return release_ok;
}

bool board_hiz_arm_break(void)
{
    return brake_ok;
}

void bridge_off(void)
{
    ++closes;
}

/* 用实测的关桥故障字和满量程读数复现入口，再注入真实故障和采样超时。 */
static void test_hiz(void)
{
    hiz_reading_t rows[3] = {0};
    hiz_reading_t idle = {.valid = true, .off = true, .fault_h = true, .guard = true,
        .raw = {.adc = {3835, 4073, 4095, 1408}, .mid = 2070, .drv_status = {9, 0, 0},
            .drv_ctrl2 = 0x70, .drv_ctrl5 = 1}};
    observed[0] = observed[1] = observed[2] = idle;
    observed[1].off = observed[1].fault_h = false;
    release_ok = true;
    assert(bench_hiz_probe(rows) == 3 && reads == 3 && releases == 1 && closes == 1);
    assert(!rows[1].off && rows[2].off && rows[0].raw.adc[2] == 4095);
    bench_hiz_report(rows, 3);
    assert(strstr(output, "BEFORE VALID=1 OFF=1") && strstr(output, "HIZ VALID=1 OFF=0") &&
        strstr(output, "RESTORED VALID=1 OFF=1") && strstr(output, "STAT=09/00/00"));
    output[0] = 0;
    observed[1].valid = false;
    reads = closes = 0;
    assert(bench_hiz_probe(rows) == 3 && !rows[1].valid && closes == 1 && rows[2].off);
    bench_hiz_report(rows, 3);
    assert(strstr(output, "PROBE INVALID"));
    output[0] = 0;
    release_ok = false;
    reads = closes = 0;
    assert(bench_hiz_probe(rows) == 1 && closes == 1);
    release_ok = true;
    for(unsigned i = 0; i < 9; ++i)
    {
        observed[0] = idle;
        switch(i)
        {
            case 0: observed[0].raw.drv_status[0] |= 0x10; break;
            case 1: observed[0].raw.drv_status[1] = 1; break;
            case 2: observed[0].raw.drv_status[2] = 1; break;
            case 3: observed[0].raw.drv_ctrl2 = 0x60; break;
            case 4: observed[0].raw.adc[3] = 262; break;
            case 5: observed[0].guard = false; break;
            case 6: observed[0].valid = false; break;
            case 7: observed[0].off = false; break;
            case 8: observed[0].raw.mid = 84; break;
        }
        reads = releases = 0;
        assert(bench_hiz_probe(rows) == 1 && reads == 1 && releases == 0);
    }
    /* 回放实板三阶段。保留位原样上报，不能把 STAT2 的第七位当作已定义故障。 */
    observed[0] = idle;
    observed[0].raw.drv_status[2] = 0x80;
    observed[1] = idle;
    observed[1].off = observed[1].fault_h = false;
    observed[1].raw.drv_status[0] = 8;
    observed[1].raw.drv_status[2] = 0x80;
    observed[1].raw.adc[0] = observed[1].raw.adc[1] = 2074;
    observed[1].raw.mid = observed[1].raw.adc[2] = 2071;
    reads = releases = 0;
    assert(bench_hiz_probe(rows) == 3 && rows[1].raw.drv_status[2] == 0x80);
    hiz_reading_t prepared;
    brake_ok = true;
    reads = releases = closes = 0;
    assert(bench_hiz_prepare(&prepared) && !prepared.off && reads == 2 && releases == 1 && !closes);
    unsigned sums[3] = {0}, count = 0;
    for(unsigned i = 0; i < 255; ++i)
        assert(!board_zero_sample(&prepared.raw, true, sums, &count));
    assert(count == 255 && !board_zero_sample(&rows[2].raw, false, sums, &count) && !count && !sums[2]);
    for(unsigned i = 1; i <= 256; ++i)
        assert(board_zero_sample(&prepared.raw, true, sums, &count) == (i == 256));
    assert(sums[0] == 2074u * 256 && sums[1] == 2074u * 256 && sums[2] == 2071u * 256);
    const hiz_reading_t good = prepared;
    for(unsigned i = 0; i < 9; ++i)
    {
        observed[1] = good;
        switch(i)
        {
            case 0: observed[1].fault_h = true; break;
            case 1: observed[1].raw.drv_status[0] |= 1; break;
            case 2: observed[1].raw.drv_status[2] |= 0x40; break;
            case 3: observed[1].raw.adc[2] = good.raw.mid + 70; break;
            case 4: observed[1].raw.adc[0] = good.raw.mid - 70; break;
            case 5: observed[1].off = true; break;
            case 6: observed[1].raw.drv_status[0] = 0; break;
            case 7: observed[1].valid = false; break;
            case 8: brake_ok = false; break;
        }
        reads = closes = 0;
        assert(!bench_hiz_prepare(&prepared) && closes == 1);
    }
}

void uart_text(const char* line)
{
    assert(strlen(output) + strlen(line) < sizeof(output));
    strcat(output, line);
}

int main(void)
{
    test_hiz();
    bench_sample_t s = {0};
    bench_report(&s);
    assert(strstr(output, "WAIT") && !strstr(output, "PASS"));
    s.ticks = 2000;
    s.configured = s.calibrated = true;
    s.sample.bus = 12.0f;
    s.live = ENCODER;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "VM PASS") && strstr(output, "DRV8316CR PASS") && strstr(output, "ADC PASS"));
    assert(strstr(output, "AS5048A FAIL") && strstr(output, "MOTOR SKIP"));
    s.raw.enc_angle_ok = s.raw.enc_diag_ok = true;
    s.raw.enc_diag = 0x100;
    s.live = SAMPLING | DRIVER | SUPPLY | DEADLINE;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "AS5048A PASS") && strstr(output, "ADC FAIL") && strstr(output, "VM FAIL"));
    assert(strstr(output, "DRV8316CR FAIL") && strstr(output, "TIMING FAIL"));
    s.off = true;
    s.live = DRIVER;
    s.raw.drv_status[0] = 9;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "DRV8316CR OFF DRVOFF=1") && strstr(output, "ADC OFF"));
    s.raw.drv_status[2] = 0x08;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "DRV8316CR FAIL"));
    s.raw.enc_diag = 0;
    s.live = ENCODER;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "AS5048A FAIL"));
    assert(bench_motor_update(MOTOR_UNTESTED, IDLE, true) == MOTOR_UNTESTED);
    assert(bench_motor_update(MOTOR_TESTING, ALIGN, false) == MOTOR_TESTING);
    assert(bench_motor_update(MOTOR_TESTING, IDLE, false) == MOTOR_STOPPED);
    assert(bench_motor_update(MOTOR_TESTING, FAULT, false) == MOTOR_FAILED);
    /* 回放电机启动后 0x001 锁存，下一拍关闭驱动时 ADC 已饱和。报告必须保留触发拍的读数。 */
    s.faults = DRIVER;
    s.mode = FAULT;
    s.raw.adc[0] = 3835;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "TRIP "));
    bench_trip_t trip = {.hw = {.site = "ENABLE_BRK", .brk = true},
        .sample = {.adc = {2075, 2073, 2074, 1391}, .mid = 2070, .drv_status = {8, 0, 0x80}},
        .elapsed = 1, .post = {9, 0, 0}, .post_ok = true};
    s.trip = &trip;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "TRIP SRC=ENABLE_BRK STEP=1 RUN=0 TP8=0 OFF=0 POEN=0 BRK=1"));
    assert(strstr(output, "TRIP SAMPLE_TP8=0 ADC=2075/2073/2074 MID=2070 VM_RAW=1391 STAT_CACHE=08/00/80"));
    assert(strstr(output, "TRIP POST_STAT=09/00/00"));
    trip.post_ok = false;
    s.raw.adc[0] = 4095;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "ADC=2075/2073/2074") && strstr(output, "TRIP POST_STAT=SPI_ERROR"));
    /* 回放 0x020 在最后一拍锁存，串口应能区分回程误差与此前通过的正扫。 */
    trip.elapsed = 8500;
    trip.alignment = (align_trace_t){.sweep = -293, .before_end = -120, .end = -75};
    s.faults = ALIGNMENT;
    s.motor = MOTOR_FAILED;
    output[0] = 0;
    bench_report(&s);
    assert(!strstr(output, "ALIGN RETURN"));
    trip.elapsed = 13000;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "ALIGN RETURN SWEEP=-293 PRE100=-120 END=-75 LIMIT=74 counts"));
    s.faults = 0;
    s.trip = NULL;
    s.motor = bench_motor_update(MOTOR_TESTING, IDLE, true);
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "MOTOR PASS"));
    assert(!strstr(output, "RPM "));
    s.rpm_active = true;
    s.speed_ref = -TAU_F / 60.0f;
    s.speed = -0.96f * TAU_F / 60.0f;
    s.iq_ref = -0.012f;
    s.iq = -0.010f;
    output[0] = 0;
    bench_report(&s);
    assert(strstr(output, "[0.01rpm]=-100/-96") && strstr(output, "[mA]=-12/-10"));
    puts("Bench: passive checks, motor reports, Hi-Z capture/restore and rejection cases passed.");
    return 0;
}
