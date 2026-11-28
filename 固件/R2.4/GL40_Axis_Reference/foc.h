/* GL40 单轴控制接口。每个电机持有一个 axis_t，角度用 rad，速度用 rad/s，电流用 A。 */
#ifndef AXIS_FOC_H
#define AXIS_FOC_H

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_HZ 10000u /* 每轴以 10 kHz 同步读反馈，外环每 10 拍执行一次。 */
#define POLE_PAIRS 14 /* GL40 KV70 每转一圈机械角，电角度转 14 圈。 */
#define PI_F 3.14159265359f
#define TAU_F (2.0f * PI_F)
#define IQ_LIMIT 0.35f /* 转矩电流给定的幅值上限，单位 A。相电流跳闸阈值另见 axis_step。 */
#define SPEED_LIMIT 4.0f /* 速度给定的幅值上限，单位 rad/s。 */
#define RPM_WINDOW_MS 8u /* 常规 rpm 测速的轴角差分窗口。 */
#define SPEED_HISTORY_MS 16u /* 单轴爬行时使用整个窗口，每 1 ms 存一次累计码。 */
#define ALIGN_RETURN_STEP 13000u /* 对齐回程结束的控制拍，1.30 s。 */
#define ALIGN_RETURN_LIMIT_RAD 0.4f /* 相对正扫起点允许的最大电角误差，单位 rad。 */

/* ALIGN 用开环电压对齐转子，JOG 用闭环转矩电流限时点动。 */
enum mode
{
    IDLE,
    ALIGN,
    JOG,
    CURRENT,
    SPEED,
    POSITION,
    FAULT
};

/* 每个原因占一位，可按位或组合。axis_t.faults 保存锁存的停机原因。 */
enum trip
{
    DRIVER = 1,
    ENCODER = 2,
    SUPPLY = 4,
    OVERCURRENT = 8,
    SAMPLING = 16,
    ALIGNMENT = 32,
    LINK_TIMEOUT = 64,
    DEADLINE = 128,
    CONFIG = 256
};

enum operation
{
    STOP,
    START_ALIGN,
    START_JOG,
    SET_IQ,
    SET_SPEED,
    SET_RPM,
    SET_POS,
    KEEP,
    GAIN,
    COG_ON,
    COG_OFF
};

enum loop_id
{
    D_LOOP,
    Q_LOOP,
    SPEED_LOOP,
    POS_LOOP
};

typedef struct
{
    enum operation op;
    enum loop_id loop; /* pid 命令选中的环。 */
    float value[3]; /* GAIN 取 Kp、Ki、Kd。运动取首项，SET_IQ 为 A，SET_POS 为 rad，两种速度命令均为 rad/s。 */
} command_t;

/* 同一套 PID 用于四个环。电流环的输入为 A、输出为 V，速度环的输出为 A，位置环的输出为 rad/s。 */
typedef struct
{
    float kp, ki, kd; /* 比例、积分、微分增益。Ki 按秒定义，pid_update 内部会乘实际采样周期。 */
    float integral; /* 已乘 Ki 的积分输出，与该环输出同单位。 */
    float previous; /* 上一拍的反馈值，用于计算测量值的变化率。 */
    float derivative; /* 低通后的反馈变化率，单位为反馈单位/s。微分项为 -Kd × derivative。 */
    bool primed; /* 已保存首个反馈样本，可在下一拍开始计算变化率。 */
} pid_tune_t;

/* 一次 PWM 同步采样交给控制器的数据。errors 中的位按 enum trip 解释。 */
typedef struct
{
    float phase[3]; /* A/B/C 三相电流，单位 A，从桥输出流向电机为正。 */
    float bus; /* 功率桥的直流母线电压，单位 V，用于保护和占空比换算。 */
    uint16_t encoder; /* AS5048A 单圈原始码，0～16383，跨整圈时回绕。 */
    unsigned errors; /* 本拍采样、驱动和时序检查得到的异常位。 */
} sample_t;

/* 同一次对齐相对正扫起点的有符号编码器位移，以跨圈累计码计算。 */
typedef struct
{
    int sweep, before_end, end; /* 正扫终点、结束前 100 ms、回程终点。 */
} align_trace_t;

/* 标定时记录正转对应的编码器方向。表项位于各格中心，单位为 mA。 */
typedef struct
{
    int direction;
    int8_t ma[512];
} axis_cog_table_t;

typedef struct
{
    enum mode mode;
    unsigned faults; /* axis_trip 锁存的故障位，复位清零。 */
    unsigned elapsed; /* 本次出力经过的控制拍数，由空闲开始运动时清零，用于对齐和点动计时。 */
    unsigned lease; /* 需要续期的模式距上次有效给定或 keep 的拍数，5000 拍触发通信超时。 */
    unsigned tick; /* 总控制拍数，空闲时也递增，用于外环分频和波形时间戳。 */
    unsigned command_result; /* 最近命令处理结果，1 接受，2 未就绪，3 拒绝。 */
    bool aligned; /* 本次对齐已取得有效方向和零点，允许进入闭环模式。 */
    bool hold_rpm; /* 单发 rpm 后保持台架速度给定，停机或故障时清零。 */
    bool encoder_seen; /* 已有可作差分基准的角度样本，首帧只建立基准。 */
    int direction; /* +1 或 -1。乘到编码器位移上，使正机械方向与正向旋转磁场一致。 */
    uint16_t last_count; /* 上一帧单圈原始码，用于识别 16383 与 0 之间的跨界。 */
    int64_t counts; /* 跨圈累加的编码器位移，单位为计数，保留编码器原始方向。 */
    int64_t origin; /* 电角对齐零点，未标定机械零位时也用作临时位置原点。 */
    int64_t align_start; /* 正扫前的 counts，与扫动终点和回程终点比较。 */
    align_trace_t align_trace;
    int64_t speed_counts[SPEED_HISTORY_MS]; /* 历史累计计数，跨过编码器单圈零点仍可直接相减。 */
    unsigned speed_slot; /* 下一次写入的位置，也是窗口中最早的采样位置。 */
    float position; /* 相对当前机械原点的连续角度，单位 rad。 */
    float speed; /* 低通后的机械角速度，单位 rad/s，1 kHz 更新。 */
    float angle; /* Park 变换使用的电角度，单位 rad，范围 [-π, π)。 */
    float target; /* 上位机原始给定，电流、速度、位置模式分别用 A、rad/s、rad。 */
    float speed_ref; /* 速度环给定，单位 rad/s，来自串口速度命令或位置环输出。 */
    float iq_ref; /* q 轴转矩电流给定，单位 A。 */
    float id, iq; /* d 轴磁链方向、q 轴转矩方向的反馈电流，单位 A。 */
    float vd, vq; /* 交给调制器的 d/q 轴电压，单位 V。闭环时来自电流 PID。 */
    float duty[3]; /* A/B/C 三相高侧占空比，0～1，供板级 PWM 驱动检查和提交。 */
    pid_tune_t pid[4]; /* 由 loop_id 索引，依次保存 d 电流、q 电流、速度和位置环的增益与历史量。 */
    int home_direction; /* 保存时的安装方向，0 表示尚未保存。 */
    uint16_t home_code; /* 机械零位对应的 AS5048A 原始单圈码。 */
    int64_t home_origin; /* 将单圈机械零位展开到当前圈的计数基准。 */
    float position_stage, position_ref; /* 两级平滑后的机械角给定，单位 rad；第二级送入位置环。 */
    uint16_t position_move_ms; /* 当前目标的短测速窗口剩余时间，由 1 kHz 速度环递减。 */
    const axis_cog_table_t* cog_table; /* 由调用方持有，须在轴的整个使用期间有效。 */
    bool cog_enabled; /* 用户开关。仅对齐通过、方向匹配的 rpm 模式施加本轴表的前馈。 */
    bool speed_pid_manual; /* 已收到 pid speed 命令，本次上电期间以用户设置为准。 */
    bool creep_rpm; /* 非零且绝对值不超过 1 rpm 的单轴速度给定。 */
    uint16_t rpm_start_ms; /* 从零进入爬行时，短测速窗口剩余的速度环拍数，每拍 1 ms。 */
    uint8_t creep_reverse_ms; /* 爬行改为相反方向后，短时缩短编码器测速窗口。 */
    float cog_current; /* 本次速度环实际叠加的转矩电流，单位 A。 */
    float previous_id, previous_iq; /* 上一拍未滤波的反馈，单位 A。 */
    float deadtime_duty[3]; /* 1 kHz 更新的三相平均占空比修正，与速度环错开计算。 */
} axis_t;

bool axis_cog_ready(const axis_t* a);
bool axis_set_home(axis_t* a, uint16_t code, int direction);
int64_t axis_position_counts(const axis_t* a);
/* 上电初始化。传 NULL 表示本轴尚未标定齿槽；只有传入该轴的标定表才会启用前馈。 */
void axis_init(axis_t* a, const axis_cog_table_t* table);
void axis_trip(axis_t* a, unsigned reason);
bool axis_command_valid(const command_t* c);
void axis_command(axis_t* a, const command_t* c, bool ready);
void axis_step(axis_t* a, const sample_t* s);
/* 可选的串口文本入口由 commands.c 提供。三轴程序可直接传入已换算为物理量的 command_t。 */
bool command_parse(char* line, command_t* c);
bool encoder_frame_ok(uint16_t word);
uint16_t spi_even_parity(uint16_t word, uint16_t parity_bit);
#endif
