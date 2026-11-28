/* 单轴 FOC 算法。axis_step 从三相电流和编码器角度算出占空比，对齐、点动和闭环共用此入口。 */
#include "foc.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DT (1.0f / CONTROL_HZ)
#define RAD_PER_COUNT (TAU_F / 16384.0f)
#define SPEED_KI_NORMAL 4.0f
#define SPEED_KI_CREEP 9.0f
#define SPEED_KI_POSITION 7.0f /* 平滑定位时及时建立、收回转矩，减少小步长脱离齿槽后的过冲。 */
#define POS_REF_ALPHA 0.018f /* 位置环每 1 ms 更新，每级时间常数约 55 ms。 */
#define POS_MOVE_MS 800u /* 大步长过渡先用 1 ms 测速，减少减速途中反馈延迟造成的振荡。 */
#define POS_HOLD_WINDOW_MS 8u /* 进入保持阶段后抑制单个编码器计数引起的电流波动。 */
#define POS_FINE_RAD (1.5f * PI_F / 180.0f) /* 静止后的小角微调用保持窗口，避免编码器量化噪声。 */
#define RPM_START_MS 350u
#define RPM_START_STEP 0.0005f /* 起步加速度 0.5 rad/s²，每 1 ms 最多改变的速度给定，rad/s。 */
#define RPM_SLEW_STEP 0.032f /* 常规调速每毫秒的最大步长，对应 32 rad/s²，两个方向共用。 */
#define RPM_CREEP_STEP 0.008f /* 爬行换向最后约 2 rpm 用较缓的斜坡，减小过零时的电流突变。 */
#define CREEP_REV_CURRENT_SLOPE 0.33f /* 爬行反向给定每改变 1 rad/s，修正的摩擦电流，单位 A。 */
#define BENCH_MAX_RPM 50.0f

/* 首个有效角度和重新对齐均建立新的测速基准，避免旧位置被当成刚发生的位移。 */
static void reset_speed_history(axis_t* a)
{
    a->speed = 0;
    for(unsigned i = 0; i < SPEED_HISTORY_MS; ++i)
    {
        a->speed_counts[i] = a->counts;
    }
}

/* 首帧或从其他模式进入定位时选择最近一圈，随后连续累计，避免回零绕行整圈。 */
static void select_home_turn(axis_t* a)
{
    int offset = (int)(((unsigned)a->last_count - a->home_code + 8192u) & 16383u) - 8192;
    a->home_origin = a->counts - offset;
}

/* 标定写入或上电恢复均在空闲时执行，防止运动过程中跳变机械位置反馈。 */
bool axis_set_home(axis_t* a, uint16_t code, int direction)
{
    if(a->mode != IDLE || code >= 16384u || (direction != 1 && direction != -1))
        return false;
    a->home_code = code;
    a->home_direction = direction;
    if(a->encoder_seen)
        select_home_turn(a);
    return true;
}

int64_t axis_position_counts(const axis_t* a)
{
    if(a->home_direction)
        return (a->counts - a->home_origin) * a->home_direction;
    return (a->counts - a->origin) * a->direction;
}

/* 对正、负两个方向使用同样的幅值限制，bound 取非负值。 */
static float limit(float value, float bound)
{
    return fmaxf(-bound, fminf(value, bound));
}

/* 计算一拍 PID 输出。ref 与 measurement 是同单位的给定和反馈，dt 单位为秒，bound 为本环输出限幅。
 * 以 q 轴为例，正误差表示实测电流低于给定，比例项会增加 Vq。实测电流上升时，微分项会抑制这次上升。 */
static float pid_update(pid_tune_t* p, float ref, float measurement, float dt, float bound, float feedforward)
{
    float error = ref - measurement;
    /* 微分取自反馈，首次为零。3 ms 低通减小编码器量化或电流采样噪声对微分项的放大。 */
    float slope = p->primed ? (measurement - p->previous) / dt : 0.0f;
    p->derivative += dt / (0.003f + dt) * (slope - p->derivative);
    p->primed = true;
    p->previous = measurement;
    float addition = p->ki * error * dt;
    float base = feedforward + p->kp * error - p->kd * p->derivative;
    float proposed = base + p->integral + addition;
    /* proposed 先试算含本拍积分增量的输出。若已越过正上限，只接受负增量回退；负下限同理。 */
    if((proposed <= bound || addition < 0) && (proposed >= -bound || addition > 0))
    {
        p->integral = limit(p->integral + addition, bound);
    }
    return limit(base + p->integral, bound);
}

/* 停机时清除动态量，同时保留已辨识的电角零点和用户设定的增益。 */
static void neutral(axis_t* a)
{
    a->hold_rpm = false;
    a->creep_rpm = false;
    a->rpm_start_ms = 0;
    a->creep_reverse_ms = 0;
    a->cog_current = 0;
    memset(a->deadtime_duty, 0, sizeof(a->deadtime_duty));
    a->target = a->iq_ref = a->speed_ref = a->vd = a->vq = 0;
    for(unsigned i = 0; i < 4; ++i)
    {
        a->pid[i].integral = a->pid[i].derivative = 0;
        a->pid[i].primed = false;
    }
    /* 三相同为 50% 时线间平均电压为零。实际关桥由驱动层执行。 */
    for(unsigned i = 0; i < 3; ++i)
    {
        a->duty[i] = 0.5f;
    }
}

/* 装入台架初始增益，复位后一律从桥关闭、待对齐的状态开始。 */
void axis_init(axis_t* a, const axis_cog_table_t* table)
{
    memset(a, 0, sizeof(*a));
    a->mode = IDLE;
    a->direction = 1;
    a->pid[D_LOOP] = a->pid[Q_LOOP] = (pid_tune_t){.kp = 0.7f, .ki = 180.0f};
    /* GL40 空载台架在 1 rpm 时需要较快积累 Iq，才能越过静止吸附位置。 */
    a->cog_table = table;
    a->cog_enabled = table != NULL;
    a->pid[SPEED_LOOP] = (pid_tune_t){.kp = 0.08f, .ki = SPEED_KI_NORMAL};
    a->pid[POS_LOOP] = (pid_tune_t){.kp = 8.0f}; /* GL40 空载小角定位，单位 (rad/s)/rad。 */
    neutral(a);
}

/* 锁存故障原因并使原电角标定失效，调用方在本次中断内关断功率桥。 */
void axis_trip(axis_t* a, unsigned reason)
{
    neutral(a);
    a->faults |= reason;
    a->mode = FAULT;
    a->aligned = false;
}

/* CAN 解包和串口解析共用相同的物理量边界。正向比较也会拒绝 NaN。 */
bool axis_command_valid(const command_t* c)
{
    switch(c->op)
    {
        case SET_IQ:
            return fabsf(c->value[0]) <= IQ_LIMIT;
        case SET_SPEED:
            return fabsf(c->value[0]) <= SPEED_LIMIT;
        case SET_RPM:
            return fabsf(c->value[0]) <= BENCH_MAX_RPM * (TAU_F / 60.0f);
        case SET_POS:
            return fabsf(c->value[0]) <= 0.5f;
        case GAIN:
            return (unsigned)c->loop <= POS_LOOP && c->value[0] >= 0 && c->value[0] <= 30 &&
                c->value[1] >= 0 && c->value[1] <= 2000 && c->value[2] >= 0 && c->value[2] <= 0.05f;
        default:
            return c->op == STOP || c->op == START_ALIGN || c->op == START_JOG ||
                c->op == KEEP || c->op == COG_ON || c->op == COG_OFF;
    }
}

/* 命令结果 1=接受，2=硬件未就绪，3=状态或参数不允许。故障锁存至复位。 */
void axis_command(axis_t* a, const command_t* c, bool ready)
{
    a->command_result = 3;
    if(!axis_command_valid(c))
        return;
    if(c->op == STOP)
    {
        neutral(a);
        if(a->mode != FAULT)
        {
            a->mode = IDLE;
        }
    }
    else if(c->op == COG_ON || c->op == COG_OFF)
    {
        if(a->mode != IDLE || (c->op == COG_ON && !axis_cog_ready(a)))
            return;
        a->cog_enabled = c->op == COG_ON;
    }
    else if(c->op == GAIN && a->mode == IDLE)
    {
        a->pid[c->loop] = (pid_tune_t){.kp = c->value[0], .ki = c->value[1], .kd = c->value[2]};
        if(c->loop == SPEED_LOOP)
            a->speed_pid_manual = true;
    }
    else if(c->op == KEEP && a->mode >= CURRENT && a->mode <= POSITION)
    {
        a->lease = 0;
    }
    else
    {
        if(!ready || a->mode == FAULT)
        {
            a->command_result = 2;
            return;
        }
        enum mode next;
        switch(c->op)
        {
            case START_ALIGN:
                next = ALIGN;
                break;
            case START_JOG:
                next = JOG;
                break;
            case SET_IQ:
                next = CURRENT;
                break;
            case SET_SPEED:
            case SET_RPM:
                next = SPEED;
                break;
            case SET_POS:
                next = POSITION;
                break;
            default:
                return;
        }
        /* 闭环需要有效的电角零点。运行中可更新同模式给定，切换控制模式须先 stop。 */
        if((next != ALIGN && !a->aligned) || (a->mode != IDLE && (a->mode != next || next < CURRENT)))
        {
            return;
        }
        if(next == POSITION && a->home_direction)
        {
            if(a->direction != a->home_direction)
                return;
            if(a->mode == IDLE)
                select_home_turn(a);
        }
        if(next == POSITION && (a->mode == IDLE || a->target != c->value[0]))
        {
            float distance = c->value[0] - (float)axis_position_counts(a) * RAD_PER_COUNT;
            bool in_transition = a->mode == POSITION && a->position_move_ms != 0;
            a->position_move_ms = (in_transition || fabsf(distance) > POS_FINE_RAD) ? POS_MOVE_MS : 0u;
        }
        if(a->mode == IDLE)
        {
            neutral(a);
            a->elapsed = 0;
            /* 关桥时转子可能回弹，首次定位从当前机械角起步；运行中改目标保留轨迹连续性。 */
            if(next == POSITION)
                a->position_stage = a->position_ref = (float)axis_position_counts(a) * RAD_PER_COUNT;
            if(next == ALIGN)
            {
                a->aligned = false;
            }
        }
        a->mode = next;
        a->hold_rpm = c->op == SET_RPM;
        /* 按指令而非瞬时测速选增益，经过慢点时不会来回切换。已有积分以 A 存储。 */
        a->creep_rpm = a->hold_rpm && c->value[0] != 0.0f && fabsf(c->value[0]) <= TAU_F / 60.0f;
        if(!a->creep_rpm)
            a->rpm_start_ms = a->creep_reverse_ms = 0;
        else if(a->target * c->value[0] < 0.0f && fabsf(a->target) <= TAU_F / 60.0f)
        {
            a->rpm_start_ms = 0;
            a->creep_reverse_ms = 120;
        }
        else if(a->target == 0.0f && fabsf(a->speed_ref) <= TAU_F / 60.0f)
            a->rpm_start_ms = RPM_START_MS;
        if(!a->speed_pid_manual)
        {
            a->pid[SPEED_LOOP].ki = a->creep_rpm ? SPEED_KI_CREEP : SPEED_KI_NORMAL;
            if(next == POSITION)
                a->pid[SPEED_LOOP].ki = SPEED_KI_POSITION;
        }
        a->lease = 0;
        a->target = c->value[0];
    }
    a->command_result = 1;
}

/* 反 Park 得到三相电压。减去最大、最小相的中点等效于对称 SVPWM 的零序注入。 */
static void modulate(axis_t* a, float cs, float sn, float bus)
{
    float alpha = a->vd * cs - a->vq * sn;
    float beta = a->vd * sn + a->vq * cs;
    float v[3] = {alpha, -0.5f * alpha + 0.8660254f * beta, -0.5f * alpha - 0.8660254f * beta};
    if(a->hold_rpm || a->mode == POSITION)
    {
        /* 爬行和小角定位时，起步、制动都需补偿桥臂死区造成的平均电压误差。
         * 由 Iq 给定重建三相电流方向。过零 40 mA 内线性过渡，避免 ADC 噪声翻转补偿方向。
         * 补偿等效 0.50 µs / 50 µs 的平均极点电压，定时器互锁死区仍为 1 µs。 */
        /* 第 7 拍更新电压修正，与第 0 拍的速度 PI 错开，给 PWM 提交留出时间。 */
        if(a->tick % 10u == 7u)
        {
            float q = 25.0f * a->iq_ref; /* 以 40 mA 为单位，免去每相一次除法。 */
            float ia = -sn * q, beta_ref = cs * q;
            float ref[3] = {ia, -0.5f * ia + 0.8660254f * beta_ref, -0.5f * ia - 0.8660254f * beta_ref};
            for(unsigned i = 0; i < 3; ++i)
                a->deadtime_duty[i] = 0.01f * (ref[i] > 1 ? 1 : ref[i] < -1 ? -1 : ref[i]);
        }
        for(unsigned i = 0; i < 3; ++i)
            v[i] += bus * a->deadtime_duty[i];
    }
    /* mid 是三相共用的电压偏移量。各相减去它以后线间电压差相同，三相占空比居中分布。 */
    float mid = 0.5f * (fmaxf(v[0], fmaxf(v[1], v[2])) + fminf(v[0], fminf(v[1], v[2])));
    for(unsigned i = 0; i < 3; ++i)
    {
        a->duty[i] = 0.5f + (v[i] - mid) / bus;
    }
}

/* 给定静止磁场，再正扫 90 电角度并回到原点。检查机械位移、回程和相电流极性。 */
static void align_step(axis_t* a, const sample_t* s)
{
    /* n 以 100 µs 为单位。field 是主动施加的磁场电角度，用它来测出转子响应的方向。 */
    unsigned n = a->elapsed;
    float field = 0.0f;
    a->vd = 0.65f * fminf(n / 1500.0f, 1.0f);
    a->vq = 0;
    /* 初始参考和末次回程都从 +90° 电角沿同一方向进入，降低齿槽造成的两侧静止位置差。 */
    if(n <= 1500)
        field = 0.5f * PI_F;
    else if(n < 3700)
        field = 0.5f * PI_F * (1.0f - (n - 1500) / 2200.0f);
    /* 500 ms 时磁场沿 A 相轴，预期 A 相流出、B/C 相流回。 */
    if(n == 5000)
    {
        if(s->phase[0] < 0.055f || s->phase[1] > -0.012f || s->phase[2] > -0.012f)
        {
            axis_trip(a, ALIGNMENT);
            return;
        }
        a->align_start = a->counts;
    }
    if(n > 5000 && n <= 8500)
    {
        field = 0.5f * PI_F * fminf((n - 5000) / 2200.0f, 1.0f);
    }
    if(n == 8500)
    {
        /* delta 把扫动期间的编码器位移折算为电弧度。幅值应接近 π/2，其符号给出安装方向。 */
        a->align_trace.sweep = (int)(a->counts - a->align_start);
        float delta = (float)a->align_trace.sweep * RAD_PER_COUNT * POLE_PAIRS;
        if(fabsf(delta) < 0.8f || fabsf(delta) > 2.35f)
        {
            axis_trip(a, ALIGNMENT);
            return;
        }
        a->direction = delta > 0 ? 1 : -1;
    }
    if(n > 8500)
    {
        field = 0.5f * PI_F * fmaxf(0, 1.0f - (n - 8500) / 2200.0f);
    }
    if(n == ALIGN_RETURN_STEP - CONTROL_HZ / 10u)
        a->align_trace.before_end = (int)(a->counts - a->align_start);
    if(n >= ALIGN_RETURN_STEP)
    {
        a->align_trace.end = (int)(a->counts - a->align_start);
        if(fabsf((float)a->align_trace.end * RAD_PER_COUNT * POLE_PAIRS) > ALIGN_RETURN_LIMIT_RAD)
        {
            axis_trip(a, ALIGNMENT);
            return;
        }
        a->origin = a->counts;
        reset_speed_history(a);
        a->position = 0;
        if(a->home_direction)
            a->position = (float)axis_position_counts(a) * RAD_PER_COUNT;
        a->aligned = true;
        a->mode = IDLE;
        neutral(a);
        return;
    }
    modulate(a, cosf(field), sinf(field), s->bus);
}

/* 仅在本轴的对齐方向与该表的标定方向一致时，按 AS5048A 原始角度施加补偿。 */
bool axis_cog_ready(const axis_t* a)
{
    return a->aligned && a->cog_table && a->cog_table->direction == a->direction;
}

/* 每格宽 32 码、表值位于格中心。减去半格后插值，跨 16383/0 时同样连续。 */
static float cogging_current(const axis_cog_table_t* table, uint16_t encoder)
{
    unsigned code = (encoder + 16384u - 16u) & 16383u;
    unsigned bin = code >> 5;
    int left = table->ma[bin];
    int right = table->ma[(bin + 1u) & 511u];
    return 0.001f * (left + (right - left) * ((code & 31u) / 32.0f));
}

/* 以 1 kHz 更新机械速度。爬行、换向和定位共用连续的编码器历史，切窗时无需重置。 */
static void estimate_speed(axis_t* a)
{
    /* 1 rpm 时 16 ms 约走 4.37 码。每毫秒更新测速，给定变化时沿用连续的角度历史。 */
    unsigned slot = a->speed_slot;
    unsigned window = a->hold_rpm ? RPM_WINDOW_MS : 1u;
    float weight = a->hold_rpm ? 0.30f : 0.15f;
    if(a->creep_rpm)
    {
        window = a->rpm_start_ms || a->creep_reverse_ms ? 4u : SPEED_HISTORY_MS;
        if(a->creep_reverse_ms)
            --a->creep_reverse_ms;
    }
    else if(a->hold_rpm && fabsf(a->target) >= TAU_F * (5.0f / 60.0f))
    {
        /* 中高速时每毫秒的编码器计数足够，缩短测速延时能更早收回制动电流。 */
        window = 2u;
        weight = 0.45f;
    }
    else if(a->mode == POSITION)
    {
        if(a->position_move_ms)
            --a->position_move_ms;
        else
            window = POS_HOLD_WINDOW_MS;
    }
    unsigned start = (slot + SPEED_HISTORY_MS - window) % SPEED_HISTORY_MS;
    float scale = RAD_PER_COUNT * (1000.0f / (float)window);
    float measured = a->direction * (float)(a->counts - a->speed_counts[start]) * scale;
    a->speed += weight * (measured - a->speed);
    a->speed_counts[slot] = a->counts;
    a->speed_slot = (slot + 1u) % SPEED_HISTORY_MS;
}

/* 以 1 kHz 推进速度给定。正反 1 rpm 换向时，按本拍斜坡增量撤回原方向的摩擦电流。 */
static void ramp_rpm(axis_t* a)
{
    float previous_ref = a->speed_ref;
    float difference = a->target - a->speed_ref;
    float step = a->rpm_start_ms ? RPM_START_STEP : RPM_SLEW_STEP;
    if(!a->rpm_start_ms && a->creep_rpm && fabsf(difference) <= 0.22f)
        step = RPM_CREEP_STEP;
    a->speed_ref = fabsf(difference) <= step
        ? a->target
        : a->speed_ref + copysignf(step, difference);
    /* 裸轴正反 1 rpm 的稳态 PI 电流相差约 83 mA。按给定的实际变化逐拍修正其中
     * 约 69 mA，使反转时原方向的摩擦电流更早退去。手调 PI 保持独立。 */
    if(a->creep_reverse_ms && !a->speed_pid_manual)
    {
        pid_tune_t* speed = &a->pid[SPEED_LOOP];
        float correction = CREEP_REV_CURRENT_SLOPE * (a->speed_ref - previous_ref);
        speed->integral = limit(speed->integral + correction, IQ_LIMIT);
    }
    if(a->rpm_start_ms && (a->rpm_start_ms > 1u || a->speed_ref == a->target))
        --a->rpm_start_ms;
}

/* 位置环跟踪两级平滑后的轨迹，将目标改变量换算为小角微调所需的速度前馈。 */
static float position_speed_ref(axis_t* a)
{
    a->position_stage += POS_REF_ALPHA * (a->target - a->position_stage);
    float change = POS_REF_ALPHA * (a->position_stage - a->position_ref);
    a->position_ref += change;
    float trajectory_speed = a->position_move_ms ? 0.0f : limit(500.0f * change, 0.055f);
    return pid_update(&a->pid[POS_LOOP], a->position_ref, a->position, 0.001f, SPEED_LIMIT, trajectory_speed);
}

/* 每次 PWM 同步采样执行一次。正相电流从桥流向电机，正 Iq 对应辨识后的正机械转向。 */
void axis_step(axis_t* a, const sample_t* s)
{
    ++a->tick;
    bool active = a->mode > IDLE && a->mode < FAULT;
    unsigned errors = s->errors;
    if(!(s->bus >= 8 && s->bus <= 15))
    {
        errors |= SUPPLY;
    }
    for(unsigned i = 0; i < 3; ++i)
    {
        if(!(fabsf(s->phase[i]) < 0.65f))
        {
            errors |= OVERCURRENT;
        }
    }
    if(active && errors)
    {
        axis_trip(a, errors);
        return;
    }
    if(errors & ENCODER)
    {
        a->encoder_seen = false;
        a->aligned = false;
        return;
    }
    /* jump 取相邻两帧的最短有符号位移。例如从 16383 到 0 为正向 1 个计数。 */
    int jump = (int)s->encoder - (int)a->last_count;
    if(jump > 8192)
    {
        jump -= 16384;
    }
    if(jump < -8192)
    {
        jump += 16384;
    }
    if(a->encoder_seen && abs(jump) > 100)
    {
        a->aligned = false;
        if(active)
        {
            axis_trip(a, ENCODER);
        }
    }
    if(a->encoder_seen)
        a->counts += jump;
    else
        reset_speed_history(a);
    a->last_count = s->encoder;
    if(!a->encoder_seen && a->home_direction)
        select_home_turn(a);
    a->position = (float)axis_position_counts(a) * RAD_PER_COUNT;
    a->encoder_seen = true;
    /* 机械位置保留累计圈数。电角度每转一对磁极就重复，乘极对数后按 16384 取模即可。 */
    uint32_t raw_electric = (uint32_t)(a->counts - a->origin) * (uint32_t)(a->direction * POLE_PAIRS);
    int32_t wrapped = (int32_t)((raw_electric + 8192u) & 16383u) - 8192;
    a->angle = wrapped * RAD_PER_COUNT;
    if(a->tick % 10u == 0)
        estimate_speed(a);
    float cs = cosf(a->angle), sn = sinf(a->angle);
    /* Clarke 将 A/B/C 三相投影到定子静止的 α/β 轴。这里的 0.57735027 为 1/√3。 */
    float alpha = (2 * s->phase[0] - s->phase[1] - s->phase[2]) / 3.0f;
    float beta = (s->phase[1] - s->phase[2]) * 0.57735027f;
    /* Park 按转子电角度旋转坐标。id 表示沿磁链的电流，iq 表示与磁链正交、产生转矩的电流。 */
    a->id = cs * alpha + sn * beta;
    a->iq = -sn * alpha + cs * beta;
    float id_feedback = a->id, iq_feedback = a->iq;
    if(a->hold_rpm && a->elapsed)
    {
        /* 10 kHz 电流反馈两点平均，群延迟 50 µs。保护和原始电流诊断始终取本拍样本。 */
        id_feedback = 0.5f * (a->id + a->previous_id);
        iq_feedback = 0.5f * (a->iq + a->previous_iq);
    }
    a->previous_id = a->id;
    a->previous_iq = a->iq;
    if(a->mode == IDLE || a->mode == FAULT)
    {
        return;
    }
    ++a->elapsed;
    if(a->mode == ALIGN)
    {
        align_step(a, s);
        return;
    }
    if(a->mode == JOG && a->elapsed >= CONTROL_HZ / 5u)
    {
        neutral(a);
        a->mode = IDLE;
        return;
    }
    if(a->mode >= CURRENT && !a->hold_rpm && ++a->lease >= CONTROL_HZ / 2u)
    {
        axis_trip(a, LINK_TIMEOUT);
        return;
    }
    /* 点动及电流模式直接给 Iq。位置模式先由位置误差求目标速度，再由速度误差求目标 Iq。
     * 两个外环每 10 拍更新一次，期间电流环沿用上一组给定。 */
    if(a->mode == JOG || a->mode == CURRENT)
    {
        a->iq_ref = a->mode == JOG ? 0.10f : a->target;
    }
    else if((a->tick % 10u) == 0)
    {
        float feedforward = 0;
        /* 给速度 PI 的 Iq 指令叠加与轴角对应的前馈，再统一执行积分抗饱和与总限流。 */
        if(a->hold_rpm && a->cog_enabled && axis_cog_ready(a))
            feedforward = cogging_current(a->cog_table, a->last_count);
        a->cog_current = feedforward;
        if(a->hold_rpm)
            ramp_rpm(a);
        else if(a->mode == POSITION)
            a->speed_ref = position_speed_ref(a);
        else
            a->speed_ref = a->target;
        a->iq_ref = pid_update(&a->pid[SPEED_LOOP], a->speed_ref, a->speed, 0.001f, IQ_LIMIT, feedforward);
    }
    /* 电压矢量留足三相低侧同时导通的采样窗口。q 轴可用电压来自 d 轴用完后的圆内余量。 */
    float voltage_ratio = 0.17f;
    if(a->hold_rpm || a->mode == POSITION)
        voltage_ratio = 0.158f; /* 给三相补偿留出共同低侧采样窗口。 */
    float vmax = fminf(2.0f, voltage_ratio * s->bus);
    a->vd = pid_update(&a->pid[D_LOOP], 0, id_feedback, DT, vmax, 0);
    float qmax = sqrtf(fmaxf(0, vmax * vmax - a->vd * a->vd));
    a->vq = pid_update(&a->pid[Q_LOOP], a->iq_ref, iq_feedback, DT, qmax, 0);
    modulate(a, cs, sn, s->bus);
}

/* 两种 SPI 芯片都采用偶校验，校验位在各自协议中的位置不同。 */
uint16_t spi_even_parity(uint16_t word, uint16_t parity_bit)
{
    uint16_t p = word;
    p ^= p >> 8;
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    return word ^ ((p & 1u) ? parity_bit : 0u);
}

/* 错误标志和偶校验均通过时，低 14 位才可作为编码器数据。 */
bool encoder_frame_ok(uint16_t word)
{
    return !(word & 0x4000u) && spi_even_parity(word, 0x8000u) == word;
}
