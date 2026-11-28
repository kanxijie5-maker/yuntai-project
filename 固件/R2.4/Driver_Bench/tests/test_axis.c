#include "../foc.h"
#include "../wave.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 用三相平均电压驱动 dq 电机模型，编码器回绕并量化到 14 位；每步调用目标固件原函数。 */
typedef struct
{
    float theta, omega; /* 模型中的机械角 rad 和机械角速度 rad/s。 */
    float d, q; /* 模型绕组的 d/q 轴电流，单位 A，用来生成控制器看到的三相反馈。 */
    int encoder_sign; /* +1 和 -1 分别模拟两种安装方向，仅改变编码器读数的方向。 */
} motor_t;

static void step(axis_t* a, motor_t* m)
{
    const float dt = 1.0f / CONTROL_HZ;
    float cs = cosf(POLE_PAIRS * m->theta), sn = sinf(POLE_PAIRS * m->theta);
    float alpha = cs * m->d - sn * m->q, beta = sn * m->d + cs * m->q;
    sample_t s = {
        .phase = {alpha, -0.5f * alpha + 0.8660254f * beta, -0.5f * alpha - 0.8660254f * beta}, .bus = 12};
    /* 加入接近单圈上限的零点偏移，使测试行程跨过 16383/0 边界，覆盖固件的角度展开逻辑。 */
    s.encoder = (uint16_t)((unsigned)lroundf(m->encoder_sign * m->theta * 16384 / TAU_F + 16360) & 16383u);
    axis_step(a, &s);
    if(a->mode == IDLE || a->mode == FAULT)
    {
        m->d *= 0.78f;
        m->q *= 0.78f;
    }
    else
    {
        for(int i = 0; i < 3; ++i)
        {
            assert(a->duty[i] >= 0.34f && a->duty[i] <= 0.66f);
        }
        alpha = 4.0f * (2 * a->duty[0] - a->duty[1] - a->duty[2]);
        beta = 12.0f * (a->duty[1] - a->duty[2]) * 0.57735027f;
        float vd = cs * alpha + sn * beta, vq = -sn * alpha + cs * beta;
        float ed = m->omega * POLE_PAIRS * 0.0009f * m->q;
        float eq = m->omega * (POLE_PAIRS * 0.0009f * m->d + 0.1f);
        m->d += dt * (vd - 2.25f * m->d + ed) / 0.0009f;
        m->q += dt * (vq - 2.25f * m->q - eq) / 0.0009f;
    }
    m->omega += dt * (0.15f * m->q - 0.002f * m->omega) / 0.00030f;
    m->theta += dt * m->omega;
}

static void cmd(axis_t* a, const char* text)
{
    command_t c;
    char line[80];
    strcpy(line, text);
    assert(command_parse(line, &c));
    axis_command(a, &c, true);
}

/* 分别以正反两种编码器方向运行同一组命令，验证对齐辨向、点动时限及各闭环的跟踪结果。 */
static void closed_loop(int sign)
{
    axis_t a;
    axis_init(&a);
    motor_t m = {.theta = 0.035f, .encoder_sign = sign};
    step(&a, &m);
    cmd(&a, "jog");
    assert(a.mode == IDLE && a.command_result == 3);
    cmd(&a, sign > 0 ? "motor" : "align");
    for(int i = 0; i < 13000; ++i)
    {
        step(&a, &m);
    }
    assert(a.mode == IDLE && a.aligned && a.direction == sign);
    /* 此处模拟理想电机，关闭实物专用的前馈以独立验证三个闭环。 */
    cmd(&a, "cogoff");
    cmd(&a, "jog");
    for(int i = 0; i < 1999; ++i)
    {
        step(&a, &m);
    }
    assert(a.mode == JOG && fabsf(a.iq - 0.10f) < 0.045f && m.omega > 0);
    step(&a, &m);
    assert(a.mode == IDLE && a.duty[0] == 0.5f);
    cmd(&a, "pid speed 0.035 0.2 0");
    assert(a.command_result == 1);
    cmd(&a, "speed -1");
    for(int i = 0; i < 18000; ++i)
    {
        if(i % 1000 == 0)
        {
            cmd(&a, "keep");
        }
        step(&a, &m);
    }
    assert(a.mode == SPEED && fabsf(a.speed + 1.0f) < 0.12f);
    /* 两个 rpm 给定各单发一次，其间没有串口续期。 */
    cmd(&a, "rpm 10");
    for(int i = 0; i < 18000; ++i)
    {
        if(i == 9000)
        {
            cmd(&a, "rpm-10");
        }
        step(&a, &m);
        if(i == 8999)
        {
            assert(a.mode == SPEED && fabsf(a.speed - TAU_F / 6.0f) < 0.12f);
        }
    }
    assert(a.mode == SPEED && fabsf(a.speed + TAU_F / 6.0f) < 0.12f);
    cmd(&a, "rpm1");
    float omega_min = 10.0f, omega_max = -10.0f;
    for(unsigned i = 0; i < CONTROL_HZ * 2u; ++i)
    {
        step(&a, &m);
        if(i >= CONTROL_HZ)
        {
            omega_min = fminf(omega_min, m.omega);
            omega_max = fmaxf(omega_max, m.omega);
        }
    }
    printf("encoder %+d: simulated shaft at 1 rpm, peak-to-peak %.5f rpm\n", sign,
        (omega_max - omega_min) * 60.0f / TAU_F);
    assert((omega_max - omega_min) * 60.0f / TAU_F < 0.02f);
    assert(a.mode == SPEED && fabsf(m.omega - TAU_F / 60.0f) < 0.04f);
    cmd(&a, "rpm-1");
    for(unsigned i = 0; i < CONTROL_HZ * 2u; ++i)
        step(&a, &m);
    assert(a.mode == SPEED && fabsf(m.omega + TAU_F / 60.0f) < 0.04f);
    cmd(&a, "stop");
    assert(a.mode == IDLE && a.target == 0.0f);
    cmd(&a, "pos 0.15");
    for(int i = 0; i < 24000; ++i)
    {
        if(i % 1000 == 0)
        {
            cmd(&a, "keep");
        }
        step(&a, &m);
    }
    assert(a.mode == POSITION && fabsf(a.position - 0.15f) < 0.02f);
    cmd(&a, "stop");
    cmd(&a, "iq -0.08");
    for(int i = 0; i < 500; ++i)
    {
        step(&a, &m);
    }
    assert(a.mode == CURRENT && fabsf(a.iq + 0.08f) < 0.02f);
    for(int i = 500; i < 5000; ++i)
    {
        step(&a, &m);
    }
    assert(a.mode == FAULT && (a.faults & LINK_TIMEOUT));
    cmd(&a, "stop");
    cmd(&a, "align");
    assert(a.mode == FAULT);
    printf("encoder %+d: align, jog, speed, position, current and timeout passed\n", sign);
}

/* 同一速度环从 rpm 改用 speed 后，从新给定起恢复 500 ms 通信超时。 */
static void rpm_timeout_switch(void)
{
    axis_t a;
    command_t c;
    char line[] = "rpm10";
    sample_t s = {.bus = 12};
    axis_init(&a);
    assert(command_parse(line, &c));
    axis_command(&a, &c, true);
    assert(a.mode == IDLE && a.command_result == 3);
    a.aligned = true;
    axis_command(&a, &c, false);
    assert(a.mode == IDLE && a.command_result == 2);
    axis_command(&a, &c, true);
    assert(a.mode == SPEED && fabsf(a.target - TAU_F / 6.0f) < 0.0001f);
    cmd(&a, "speed 1");
    for(unsigned i = 1; i < CONTROL_HZ / 2u; ++i)
    {
        axis_step(&a, &s);
    }
    assert(a.mode == SPEED);
    axis_step(&a, &s);
    assert(a.mode == FAULT && a.faults == LINK_TIMEOUT);
    axis_command(&a, &c, true);
    assert(a.mode == FAULT && a.command_result == 2 && a.target == 0.0f);
    axis_init(&a);
    assert(a.mode == IDLE && !a.aligned && a.target == 0.0f);
}

/* 回放正扫成功、回程刚好在界限两侧的角度序列，两种方向均经过编码器单圈零点。 */
static void alignment_return(int sign, int residual)
{
    axis_t a;
    axis_init(&a);
    cmd(&a, "motor");
    sample_t s = {.phase = {0.2f, -0.1f, -0.1f}, .bus = 12};
    for(int n = 1; n <= 13000; ++n)
    {
        int offset = 0;
        if(n > 12000)
            offset = 120 + (residual - 120) * (n - 12000) / 1000;
        else if(n > 8500)
            offset = 293 - (293 - 120) * (n - 8500) / 3500;
        else if(n > 5000)
            offset = 293 * (n - 5000) / 3500;
        s.encoder = (uint16_t)(((sign > 0 ? 16380 : 4) + sign * offset) & 16383);
        axis_step(&a, &s);
    }
    assert(a.align_trace.sweep == sign * 293 && a.align_trace.before_end == sign * 120);
    assert(a.align_trace.end == sign * residual);
    if(residual == 74)
        assert(a.mode == IDLE && a.aligned && !a.faults);
    else
        assert(a.mode == FAULT && !a.aligned && a.faults == ALIGNMENT);
}

/* 向运行中的控制器注入故障，并检查对齐受阻、输出饱和和反向给定时的处理。 */
static void protection(void)
{
    const unsigned flags[] = {DRIVER, ENCODER, SAMPLING, DEADLINE, CONFIG, OVERCURRENT, SUPPLY};
    for(unsigned i = 0; i < sizeof(flags) / sizeof(flags[0]); ++i)
    {
        axis_t a;
        axis_init(&a);
        a.aligned = true;
        cmd(&a, "rpm10");
        sample_t s = {.bus = 12};
        if(flags[i] == SUPPLY)
        {
            s.bus = 16;
        }
        else if(flags[i] == OVERCURRENT)
        {
            s.phase[2] = -0.68f;
        }
        else
        {
            s.errors = flags[i];
        }
        axis_step(&a, &s);
        assert(a.mode == FAULT && a.faults == flags[i] && a.duty[2] == 0.5f);
        assert(!a.hold_rpm && a.target == 0.0f);
    }
    axis_t a;
    axis_init(&a);
    /* 电机未接时，零相电流不能通过通电检测；即使编码器给出固定的有效角度也一样。 */
    cmd(&a, "motor");
    sample_t unplugged = {.bus = 12};
    for(unsigned i = 0; i < 5000; ++i)
        axis_step(&a, &unplugged);
    assert(a.mode == FAULT && a.faults == ALIGNMENT && !a.aligned);
    axis_init(&a);
    cmd(&a, "align");
    sample_t blocked = {.phase = {0.2f, -0.1f, -0.1f}, .bus = 12};
    for(unsigned i = 0; i < 8500; ++i)
    {
        axis_step(&a, &blocked);
    }
    assert(a.mode == FAULT && a.faults == ALIGNMENT);
    axis_init(&a);
    a.aligned = true;
    cmd(&a, "iq 0.35");
    sample_t no_current = {.bus = 8};
    for(unsigned i = 0; i < 4000; ++i)
    {
        axis_step(&a, &no_current);
    }
    assert(a.vq <= 1.361f && fabsf(a.pid[Q_LOOP].integral) < 1.36f);
    cmd(&a, "iq -0.35");
    for(unsigned i = 0; i < 1000; ++i)
    {
        axis_step(&a, &no_current);
    }
    assert(a.vq < -1.2f);
}

/* 模拟轴被负载固定，检查串口给定的正、负转矩电流是否持续跟踪，失联后是否关桥。 */
static void torque_at_standstill(void)
{
    axis_t a;
    axis_init(&a);
    a.aligned = true;
    motor_t m = {.encoder_sign = 1};
    for(int sign = 1; sign >= -1; sign -= 2)
    {
        for(unsigned i = 0; i < CONTROL_HZ; ++i)
        {
            if(i % 1000u == 0)
            {
                cmd(&a, sign > 0 ? "torque 0.10" : "torque -0.10");
            }
            m.theta = m.omega = 0;
            step(&a, &m);
        }
        assert(a.mode == CURRENT && fabsf(a.iq - sign * 0.10f) < 0.01f);
    }
    cmd(&a, "keep");
    sample_t s = {.bus = 12, .encoder = a.last_count};
    for(unsigned i = 0; i < CONTROL_HZ / 2u; ++i)
    {
        axis_step(&a, &s);
    }
    assert(a.mode == FAULT && a.faults == LINK_TIMEOUT);
}

/* 验证无效命令会被拒绝、编码器能识别翻转的一位，以及波形首尾在串口上的字节顺序。 */
static void protocol(void)
{
    const char* invalid[] = {"iq nan", "speed inf", "pos -0.51", "pid q 0 -1 0", "pid d 1 2", "pid x 1 2 3",
        "pid q 1 3 1", "iq 1e90", "jog 1", "keep 0", "iq 0.2V", "", "speed", "STOP", "rpm", "rpm inf",
        "rpm 38.2", "rpm -38.2", "rpm 10 5", "rpm38.2", "rpm--1", "rpm10 5", "rpm10x", "torque", "torque nan",
        "torque 0.351", "torque -0.351", "torque 0.05 1", "motor 1", "cogcal", "trial 21", "cogset 100 0",
        "cognoise", "cogmap", "cogon 1"};
    for(unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        command_t c;
        char line[80];
        strcpy(line, invalid[i]);
        assert(!command_parse(line, &c));
    }
    assert(spi_even_parity(0x7fff, 0x8000) == 0xffff);
    assert(spi_even_parity(0x8000, 0x0100) == 0x8100);
    for(uint16_t i = 0; i < 16384; ++i)
    {
        uint16_t frame = spi_even_parity(i, 0x8000);
        assert(encoder_frame_ok(frame) && !encoder_frame_ok(frame ^ 8u));
    }
    assert(!encoder_frame_ok(0xffff) && !encoder_frame_ok(0xc000));
    float ch[CHANNELS] = {1.0f};
    ch[CHANNELS - 1] = -2.0f;
    wave_frame_t frame;
    wave_pack(&frame, ch);
    const uint8_t* bytes = (const uint8_t*)&frame;
    const uint8_t first[] = {0, 0, 128, 63}, last[] = {0, 0, 0, 192, 0, 0, 128, 127};
    assert(!memcmp(bytes, first, 4) && !memcmp(bytes + sizeof(frame) - 8, last, 8));
}

/* 回放理想匀速轴的 14 位编码器：位置先取整再回绕，每毫秒记录实际供速度环使用的反馈。 */
static void low_speed_feedback(float rpm)
{
    axis_t a;
    axis_init(&a);
    a.aligned = true;
    command_t c = {.op = SET_RPM, .value = {rpm * TAU_F / 60.0f}};
    axis_command(&a, &c, true);
    sample_t s = {.bus = 12};
    float lo = 100.0f, hi = -100.0f, total = 0.0f;
    unsigned samples = 0;
    for(unsigned i = 1; i <= CONTROL_HZ * 2u; ++i)
    {
        long count = (rpm > 0 ? 16380 : 4) + lround((double)i * rpm * 16384.0 / (60.0 * CONTROL_HZ));
        s.encoder = (uint16_t)((unsigned long)count & 16383u);
        axis_step(&a, &s);
        if(i > CONTROL_HZ / 2u && i % 10u == 0)
        {
            float measured = a.speed * 60.0f / TAU_F;
            lo = fminf(lo, measured);
            hi = fmaxf(hi, measured);
            total += measured;
            ++samples;
        }
    }
    printf(
        "encoder input %+.2f rpm: speed mean %.4f, peak-to-peak %.4f rpm\n", rpm, total / samples, hi - lo);
    fflush(stdout);
    assert(a.mode == SPEED && fabsf(total / samples - rpm) < 0.01f && hi - lo < 0.25f);
    /* 随后保持编码器静止，检查旧位移能按时退出估算窗口。 */
    for(unsigned i = 0; i < CONTROL_HZ / 20u; ++i)
        axis_step(&a, &s);
    assert(fabsf(a.speed) < 0.001f);
}

/* 用 D 组记录独立算出的校验值，覆盖每个格中心、格间插值及整圈接缝。 */
static void fixed_cogging_test(void)
{
    axis_t a;
    sample_t s = {.bus = 12};
    axis_init(&a);
    cmd(&a, "cogon");
    assert(!axis_cog_ready(&a) && a.command_result == 3);
    cmd(&a, "rpm1");
    assert(a.mode == IDLE);
    a.aligned = true;
    cmd(&a, "pid speed 0 0 0");
    cmd(&a, "rpm1");
    cmd(&a, "cogoff");
    assert(a.command_result == 3 && a.cog_enabled);
    uint32_t checksum = 2166136261u;
    for(unsigned code = 0; code < 16384; ++code)
    {
        s.encoder = (uint16_t)code;
        for(unsigned n = 0; n < 10; ++n)
            axis_step(&a, &s);
        /* 插值的最小步长为 1/32 mA，将安培值还原为整数后再校验。 */
        int q32 = (int)lroundf(a.cog_current * 32000.0f);
        checksum = (checksum ^ (uint32_t)q32) * 16777619u;
        assert(a.mode == SPEED && a.iq_ref == a.cog_current);
    }
    assert(checksum == 0x5eebba65u);
    cmd(&a, "stop");
    cmd(&a, "cogoff");
    cmd(&a, "rpm-1");
    for(unsigned n = 0; n < 10; ++n)
        axis_step(&a, &s);
    assert(a.cog_current == 0.0f && a.iq_ref == 0.0f);
    cmd(&a, "stop");
    cmd(&a, "cogon");
    assert(a.command_result == 1 && a.cog_enabled);
    a.direction = -1;
    cmd(&a, "cogon");
    assert(!axis_cog_ready(&a) && a.command_result == 3);
    cmd(&a, "rpm1");
    for(unsigned n = 0; n < 10; ++n)
        axis_step(&a, &s);
    assert(a.mode == SPEED && a.cog_current == 0.0f);
    axis_trip(&a, OVERCURRENT);
    assert(!axis_cog_ready(&a) && a.cog_current == 0.0f);
}

/* 核对固定的死区补偿、相邻两拍电流平均以及未经平均的过流保护。 */
static void low_speed_current_feedback(void)
{
    axis_t a;
    for(int sign = -1; sign <= 1; sign += 2)
    {
        axis_init(&a);
        cmd(&a, "cogoff");
        a.aligned = true;
        cmd(&a, "rpm1");
        a.pid[Q_LOOP] = (pid_tune_t){0};
        a.iq_ref = sign * 0.1f;
        a.tick = 6; /* 第 7 拍更新三相补偿。 */
        sample_t s = {.bus = 12};
        axis_step(&a, &s);
        assert(fabsf(a.duty[1] - a.duty[2] - sign * 0.02f) < 0.0001f);
    }
    axis_init(&a);
    cmd(&a, "cogoff");
    a.aligned = true;
    cmd(&a, "rpm1");
    a.pid[Q_LOOP] = (pid_tune_t){.kp = 1};
    sample_t s = {.bus = 12};
    axis_step(&a, &s);
    s.phase[1] = 0.173205f;
    s.phase[2] = -s.phase[1];
    axis_step(&a, &s);
    assert(fabsf(a.iq - 0.2f) < 0.0001f && fabsf(a.vq + 0.1f) < 0.0001f);
    s.phase[2] = 0.66f;
    axis_step(&a, &s);
    assert(a.mode == FAULT && (a.faults & OVERCURRENT));
}

int main(void)
{
    low_speed_current_feedback();
    fixed_cogging_test();
    protocol();
    for(int sign = -1; sign <= 1; sign += 2)
    {
        alignment_return(sign, 74);
        alignment_return(sign, 75);
    }
    protection();
    rpm_timeout_switch();
    torque_at_standstill();
    puts("Signed torque current at standstill and serial timeout passed");
    closed_loop(1);
    closed_loop(-1);
    low_speed_feedback(1.0f);
    low_speed_feedback(-1.0f);
    low_speed_feedback(0.5f);
    low_speed_feedback(1.3f);
    puts("PID limits, fault handling, SPI parity, command rejection and JustFloat bytes passed");
    return 0;
}
