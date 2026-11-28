#include "../foc.h"
#include "../home_record.h"
#include "../gl40_cogging_table.h"
#include "../current_zero.h"
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
    axis_init(&a, NULL);
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
    axis_init(&a, NULL);
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
    axis_init(&a, NULL);
    assert(a.mode == IDLE && !a.aligned && a.target == 0.0f);
}

/* 回放正扫成功、回程刚好在界限两侧的角度序列，两种方向均经过编码器单圈零点。 */
static void alignment_return(int sign, int residual)
{
    axis_t a;
    axis_init(&a, NULL);
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
        axis_init(&a, NULL);
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
    axis_init(&a, NULL);
    /* 电机未接时，零相电流不能通过通电检测；即使编码器给出固定的有效角度也一样。 */
    cmd(&a, "motor");
    sample_t unplugged = {.bus = 12};
    for(unsigned i = 0; i < 5000; ++i)
        axis_step(&a, &unplugged);
    assert(a.mode == FAULT && a.faults == ALIGNMENT && !a.aligned);
    axis_init(&a, NULL);
    cmd(&a, "align");
    sample_t blocked = {.phase = {0.2f, -0.1f, -0.1f}, .bus = 12};
    for(unsigned i = 0; i < 8500; ++i)
    {
        axis_step(&a, &blocked);
    }
    assert(a.mode == FAULT && a.faults == ALIGNMENT);
    axis_init(&a, NULL);
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
    axis_init(&a, NULL);
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

/* 验证无效命令会被拒绝，编码器能识别翻转的一位。 */
static void protocol(void)
{
    const char* invalid[] = {"iq nan", "speed inf", "pos -0.51", "pid q 0 -1 0", "pid d 1 2", "pid x 1 2 3",
        "pid q 1 3 1", "iq 1e90", "jog 1", "keep 0", "iq 0.2V", "", "speed", "STOP", "rpm", "rpm inf",
        "rpm 50.01", "rpm -50.01", "rpm 10 5", "rpm51", "speed 4.01", "rpm--1", "rpm10 5", "rpm10x", "torque", "torque nan",
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
}

/* 回放理想匀速轴的 14 位编码器：位置先取整再回绕，每毫秒记录实际供速度环使用的反馈。 */
static void low_speed_feedback(float rpm)
{
    axis_t a;
    axis_init(&a, NULL);
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
    float ripple_limit = fabsf(rpm) <= 1.0f ? 0.15f : 0.25f;
    assert(a.mode == SPEED && fabsf(total / samples - rpm) < 0.01f && hi - lo < ripple_limit);
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
    axis_init(&a, &gl40_table_d);
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
    const char* modes[] = {"rpm1", "pos 0.017453"};
    for(unsigned mode = 0; mode < sizeof(modes) / sizeof(modes[0]); ++mode)
    {
        for(int sign = -1; sign <= 1; sign += 2)
        {
            axis_init(&a, NULL);
            cmd(&a, "cogoff");
            a.aligned = true;
            cmd(&a, modes[mode]);
            a.pid[Q_LOOP] = (pid_tune_t){0};
            a.iq_ref = sign * 0.1f;
            a.tick = 6; /* 第 7 拍更新三相补偿。 */
            sample_t s = {.bus = 12};
            axis_step(&a, &s);
            assert(fabsf(a.duty[1] - a.duty[2] - sign * 0.02f) < 0.0001f);
            cmd(&a, "stop");
            assert(a.deadtime_duty[1] == 0 && a.deadtime_duty[2] == 0);
        }
    }
    axis_init(&a, NULL);
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

/* 台架爬行速度使用实板验证的积分增益；切换给定时不清已有积分，手动调参优先。 */
static void rpm_creep_gain(void)
{
    axis_t a;
    axis_init(&a, NULL);
    a.aligned = true;
    cmd(&a, "rpm1");
    float creep_ki = a.pid[SPEED_LOOP].ki;
    assert(creep_ki == 9.0f);
    cmd(&a, "rpm-1");
    assert(a.pid[SPEED_LOOP].ki == creep_ki);
    a.pid[SPEED_LOOP].integral = 0.03f;
    cmd(&a, "rpm10");
    assert(a.pid[SPEED_LOOP].ki == 4.0f && a.pid[SPEED_LOOP].integral == 0.03f);
    cmd(&a, "rpm0");
    assert(a.pid[SPEED_LOOP].ki == 4.0f);
    cmd(&a, "rpm0.5");
    assert(a.pid[SPEED_LOOP].ki == creep_ki);
    cmd(&a, "stop");
    cmd(&a, "pos 0");
    assert(a.pid[SPEED_LOOP].ki == 7.0f);
    cmd(&a, "stop");
    cmd(&a, "pid speed 0.07 3 0");
    cmd(&a, "rpm1");
    assert(a.pid[SPEED_LOOP].ki == 3.0f);
    cmd(&a, "stop");
    cmd(&a, "pos 0");
    assert(a.pid[SPEED_LOOP].ki == 3.0f);
}

/* 从关桥和零速保持分别向两侧起步。重复同一个 rpm 命令不能重新启动斜坡。 */
static void rpm_start_from_rest(void)
{
    axis_t a;
    axis_init(&a, NULL);
    a.aligned = true;
    sample_t s = {.bus = 12};
    for(int sign = 1; sign >= -1; sign -= 2)
    {
        const char* command = sign > 0 ? "rpm1" : "rpm-1";
        cmd(&a, command);
        for(unsigned ms = 1; ms <= 360; ++ms)
        {
            if(ms == 60)
                cmd(&a, command);
            for(unsigned tick = 0; tick < 10; ++tick)
                axis_step(&a, &s);
            float expected = fminf(0.0005f * ms, TAU_F / 60.0f);
            assert(fabsf(a.speed_ref - sign * expected) < 0.00001f);
            assert(a.rpm_start_ms == (ms < 350 ? 350u - ms : 0u));
        }
        cmd(&a, "rpm0");
        for(unsigned tick = 0; tick < 400; ++tick)
            axis_step(&a, &s);
        assert(a.speed_ref == 0);
    }
    cmd(&a, "stop");
    cmd(&a, "rpm10");
    for(unsigned tick = 0; tick < 10; ++tick)
        axis_step(&a, &s);
    assert(a.speed_ref > 0 && a.speed_ref < a.target);
}

/* 连续调速经过零速时，两侧限加速度相同。重复命令不得重启轨迹，stop 仍须立即撤力。 */
static void rpm_reverse(void)
{
    axis_t a;
    sample_t s = {.bus = 12};
    axis_init(&a, NULL);
    a.aligned = true;
    cmd(&a, "rpm50");
    for(unsigned i = 0; i < CONTROL_HZ; ++i)
        axis_step(&a, &s);
    assert(a.speed_ref == a.target && a.speed_ref > 5.23f);
    for(int sign = -1; sign <= 1; sign += 2)
    {
        const char* command = sign < 0 ? "rpm-50" : "rpm50";
        cmd(&a, command);
        for(unsigned ms = 0; ms < 1000; ++ms)
        {
            float before = a.speed_ref;
            if(ms == 20)
                cmd(&a, command);
            for(unsigned tick = 0; tick < 10; ++tick)
                axis_step(&a, &s);
            float step = sign * (a.speed_ref - before);
            assert(step >= 0 && step <= 0.04f);
        }
        assert(a.speed_ref == a.target);
    }
    cmd(&a, "stop");
    assert(a.mode == IDLE && a.speed_ref == 0 && a.iq_ref == 0);
}

/* 正反小阶跃都可借用轨迹速度，到位后前馈消失，不改变静态位置增益。 */
static void small_position_feedforward(void)
{
    sample_t s = {.bus = 12};
    for(int sign = -1; sign <= 1; sign += 2)
    {
        axis_t a;
        axis_init(&a, NULL);
        a.aligned = true;
        cmd(&a, sign < 0 ? "pos -0.017453" : "pos 0.017453");
        float peak = 0;
        for(unsigned i = 0; i < CONTROL_HZ; ++i)
        {
            axis_step(&a, &s);
            float correction = sign * (a.speed_ref - a.pid[POS_LOOP].kp * a.position_ref);
            assert(correction >= -0.00001f && correction <= 0.056f);
            peak = fmaxf(peak, correction);
            if(i % 1000u == 0u)
                cmd(&a, "keep");
        }
        assert(peak > 0.02f && fabsf(a.speed_ref - a.pid[POS_LOOP].kp * a.position_ref) < 0.0001f);
    }
}

/* 同一组速度误差下，自动参数会逐拍撤去旧方向的摩擦电流，手调 PI 则只积分误差。 */
static void creep_direction_change(void)
{
    sample_t s = {.bus = 12};
    axis_t automatic;
    axis_init(&automatic, NULL);
    automatic.aligned = true;
    cmd(&automatic, "rpm1");
    automatic.speed_ref = automatic.target;
    cmd(&automatic, "rpm-1");
    axis_t manual = automatic;
    manual.speed_pid_manual = true;
    for(unsigned ms = 0; ms < 120; ++ms)
    {
        if(ms == 5)
            cmd(&automatic, "rpm-1");
        for(unsigned i = 0; i < 10; ++i)
        {
            axis_step(&automatic, &s);
            axis_step(&manual, &s);
        }
        assert(automatic.creep_reverse_ms == 119u - ms);
        if(ms == 15)
        {
            float extra = manual.pid[SPEED_LOOP].integral - automatic.pid[SPEED_LOOP].integral;
            assert(extra > 0.01f && extra < 0.07f);
        }
    }
    cmd(&automatic, "rpm1");
    assert(automatic.creep_reverse_ms > 0);
    cmd(&automatic, "stop");
    assert(automatic.creep_reverse_ms == 0 && automatic.pid[SPEED_LOOP].integral == 0);
}

/* 从不同上电角进入位置模式，跨过编码器接缝后仍回到已保存的机械零点。 */
static void mechanical_home(void)
{
    axis_t a;
    home_record_t slots[2] = {{UINT32_MAX, UINT32_MAX}, {UINT32_MAX, UINT32_MAX}};
    assert(home_latest(slots) == -1);
    slots[0].value = 0xffff8004u;
    assert(home_latest(slots) == -1);
    slots[0].inverse = ~slots[0].value;
    slots[1].value = 0x0000c006u;
    assert(home_latest(slots) == 0);
    slots[1].inverse = ~slots[1].value;
    assert(home_latest(slots) == 1);
    slots[0].value = 0x00018012u;
    slots[0].inverse = ~slots[0].value;
    assert(home_latest(slots) == 0);
    for(int sign = -1; sign <= 1; sign += 2)
    {
        axis_init(&a, NULL);
        axis_set_home(&a, 16380, sign);
        sample_t s = {.bus = 12, .encoder = 3};
        axis_step(&a, &s);
        assert(axis_position_counts(&a) == sign * 7);
        a.direction = sign;
        a.aligned = true;
        a.origin = a.counts + 21;
        cmd(&a, "pos 0");
        assert(a.mode == POSITION && axis_position_counts(&a) == sign * 7);
        s.encoder = 16379;
        axis_step(&a, &s);
        assert(axis_position_counts(&a) == -sign && a.origin == 21);
        cmd(&a, "stop");
        a.counts += 16384;
        cmd(&a, "pos 0");
        assert(axis_position_counts(&a) == -sign);
        cmd(&a, "stop");
        a.direction = -sign;
        char line[] = "pos 0";
        command_t c;
        assert(command_parse(line, &c));
        axis_command(&a, &c, true);
        assert(a.mode == IDLE && a.command_result == 3);
    }
}

/* 模拟轴暂时静止。从非零机械角启用、运动中改目标，速度给定都应连续；稳定后仍达原给定。 */
static void position_command_transition(void)
{
    axis_t a;
    sample_t s = {.bus = 12, .encoder = 1040};
    axis_init(&a, NULL);
    axis_set_home(&a, 1000, 1);
    axis_step(&a, &s);
    a.aligned = true;
    cmd(&a, "pos 0.18");
    for(unsigned i = 0; i < 10; ++i)
        axis_step(&a, &s);
    assert(a.speed_ref > 0 && a.speed_ref < 0.03f);
    for(unsigned i = 0; i < 120; ++i)
        axis_step(&a, &s);
    float before = a.speed_ref;
    cmd(&a, "pos -0.10");
    for(unsigned i = 0; i < 10; ++i)
        axis_step(&a, &s);
    assert(fabsf(a.speed_ref - before) < 0.08f);
    for(unsigned i = 0; i < 10000; ++i)
    {
        if(i % 1000 == 0)
            cmd(&a, "pos -0.10"); /* 上位机重复同一目标时也应继续跟踪，不能重新起步。 */
        axis_step(&a, &s);
    }
    assert(a.mode == POSITION && fabsf(a.speed_ref - a.pid[POS_LOOP].kp * (-0.10f - a.position)) < 0.001f);
    cmd(&a, "stop");
    assert(a.mode == IDLE && a.speed_ref == 0 && a.duty[0] == 0.5f);
}

/* 用编码器一格位移检验测速延迟：改目标后的响应要快，保持时同一格噪声的影响应减小。 */
static void position_speed_windows(void)
{
    axis_t a;
    sample_t s = {.bus = 12};
    axis_init(&a, NULL);
    a.aligned = true;
    cmd(&a, "pos 0.05");
    for(unsigned ms = 0; ms < 1200; ++ms)
    {
        if(ms % 100 == 0 && ms <= 1000)
            cmd(&a, "pos 0.05"); /* 重发相同目标不应重置保持阶段。 */
        if(ms == 1080)
            cmd(&a, "pos 0.015"); /* 实际轴仍在零位，静止后给一个不足 1° 的微调。 */
        s.encoder = (ms == 100 || ms == 1000 || ms == 1080 || ms == 1199);
        if(ms == 1199)
            cmd(&a, "pos -0.05");
        for(unsigned i = 0; i < 10; ++i)
            axis_step(&a, &s);
        if(s.encoder)
        {
            float expected = 0.15f * TAU_F * 1000.0f / 16384.0f;
            if(ms == 1000 || ms == 1080)
                expected /= 8;
            assert(fabsf(a.speed - expected) < 0.00001f);
        }
    }
}

/* 两次取得 A 相参考点前均从正电角一侧接近，减少不同方向到达齿槽时的静止偏差。 */
static void alignment_approach(void)
{
    axis_t a;
    axis_init(&a, NULL);
    cmd(&a, "motor");
    sample_t s = {.bus = 12};
    for(unsigned i = 0; i < 1500; ++i)
        axis_step(&a, &s);
    assert(a.mode == ALIGN && a.duty[1] > a.duty[0] && a.duty[0] > a.duty[2]);
    for(unsigned i = 1500; i < 4000; ++i)
        axis_step(&a, &s);
    assert(a.duty[0] > a.duty[1] && fabsf(a.duty[1] - a.duty[2]) < 0.0001f);
}

/* 三路在同一节拍交错更新，检查角度、补偿、故障和积分均属于各自的轴。 */
static void three_axes(void)
{
    axis_t a[3];
    static const axis_cog_table_t reverse_table = {.direction = -1, .ma = {12}};
    const axis_cog_table_t* tables[] = {&gl40_table_d, NULL, &reverse_table};
    sample_t s[3] = {{.encoder = 16, .bus = 12}, {.encoder = 8000, .bus = 12}, {.encoder = 16, .bus = 12}};
    for(unsigned i = 0; i < 3; ++i)
    {
        axis_init(&a[i], tables[i]);
        axis_step(&a[i], &s[i]);
        a[i].aligned = true;
        a[i].direction = i == 2 ? -1 : 1;
        assert(axis_set_home(&a[i], s[i].encoder, a[i].direction));
        cmd(&a[i], "rpm1");
    }
    for(unsigned n = 1; n < 30; ++n)
        for(unsigned i = 0; i < 3; ++i)
            axis_step(&a[i], &s[i]);
    assert(fabsf(a[0].cog_current + 0.007f) < 1e-6f && a[1].cog_current == 0);
    assert(fabsf(a[2].cog_current - 0.012f) < 1e-6f);
    assert(!axis_set_home(&a[0], 100, 1) && a[0].home_code == 16);
    axis_trip(&a[2], ENCODER);
    cmd(&a[1], "stop");
    assert(a[2].mode == FAULT && a[1].mode == IDLE && a[0].mode == SPEED);
    assert(a[0].pid[SPEED_LOOP].integral > 0 && a[2].pid[SPEED_LOOP].integral == 0);
    assert(!axis_set_home(&a[1], 16384u, 1) && !axis_set_home(&a[1], 25, 0));
    assert(a[1].home_code == 8000 && !axis_cog_ready(&a[1]));
}

static void direct_command_limits(void)
{
    axis_t a;
    axis_init(&a, NULL);
    a.aligned = true;
    const command_t invalid[] = {{.op = SET_IQ, .value = {0.5f}}, {.op = SET_SPEED, .value = {NAN}},
        {.op = SET_POS, .value = {INFINITY}}, {.op = SET_RPM, .value = {-6.0f}},
        {.op = GAIN, .loop = (enum loop_id)4}, {.op = GAIN, .loop = SPEED_LOOP, .value = {0, NAN, 0}}};
    for(unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        axis_command(&a, &invalid[i], true);
        assert(a.mode == IDLE && a.command_result == 3);
    }
    command_t c = {.op = SET_IQ, .value = {-IQ_LIMIT}};
    axis_command(&a, &c, true);
    assert(a.mode == CURRENT && a.command_result == 1 && a.target == -IQ_LIMIT);
    a.lease = 24;
    axis_command(&a, &invalid[0], true);
    assert(a.lease == 24 && a.target == -IQ_LIMIT);
}

/* 回放关桥时的近满量程读数，以及拉低 DRVOFF 后的正常中点读数。 */
static void adc_offset(void)
{
    current_zero_t z = {0};
    uint16_t adc[3] = {2074, 2073, 2071};
    for(unsigned i = 0; i < 255; ++i)
        assert(!current_zero_sample(&z, adc, 2070, true));
    assert(!current_zero_sample(&z, adc, 2070, false) && z.count == 0);
    const uint16_t off[3] = {3836, 4074, 4095};
    assert(!current_zero_sample(&z, off, 2070, true));
    assert(!current_zero_sample(&z, adc, 1600, true));
    for(unsigned i = 1; i <= 256; ++i)
        assert(current_zero_sample(&z, adc, 2070, true) == (i == 256));
    assert(current_zero_sample(&z, adc, 2070, true) && z.count == 256);
    for(unsigned i = 0; i < 3; ++i)
        assert(z.sum[i] == 256u * adc[i]);
}

int main(void)
{
    three_axes();
    direct_command_limits();
    adc_offset();
    rpm_reverse();
    small_position_feedforward();
    creep_direction_change();
    alignment_approach();
    mechanical_home();
    position_command_transition();
    position_speed_windows();
    rpm_creep_gain();
    rpm_start_from_rest();
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
    puts("PID limits, fault handling, SPI parity, command rejection passed");
    return 0;
}
