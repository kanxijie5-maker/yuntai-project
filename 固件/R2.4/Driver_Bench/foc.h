/* 单轴控制器的数据接口。调试时可从这里把变量的单位、正方向与 VOFA+ 通道对应起来。 */
#ifndef AXIS_FOC_H
#define AXIS_FOC_H

#include <stdbool.h>
#include <stdint.h>

#include "timing.h"
#define POLE_PAIRS 14 /* GL40 KV70 每转一圈机械角，电角度转 14 圈。 */
#define PI_F 3.14159265359f
#define TAU_F (2.0f * PI_F)
#define IQ_LIMIT 0.35f /* 转矩电流给定的幅值上限，单位 A。相电流跳闸阈值另见 axis_step。 */
#define SPEED_LIMIT 4.0f /* 速度给定的幅值上限，单位 rad/s。 */
#define RPM_WINDOW_MS 8u /* 每毫秒保存一次轴角，rpm 模式用最近 8 ms 的位移估速。 */
#define ALIGN_RETURN_STEP 13000u /* 对齐回程结束的控制拍，1.30 s。 */
#define ALIGN_RETURN_LIMIT_RAD 0.4f /* 相对正扫起点允许的最大电角误差，单位 rad。 */

/* CH1 显示下列状态的数值。ALIGN 用开环电压对齐转子，JOG 用闭环转矩电流限时点动。 */
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

/* 每个原因占一位，可按位或组合。CH3 显示本拍异常，CH2 显示锁存的停机原因。 */
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
    float value[3]; /* pid 命令依次为 Kp、Ki、Kd。运动命令只用 value[0] 保存给定。 */
} command_t;

/* 同一套 PID 用于四个环。电流环的输入为 A、输出为 V，速度环的输出为 A，位置环的输出为 rad/s。 */
typedef struct
{
    float kp, ki, kd; /* 比例、积分、微分增益。Ki 按秒定义，pid_update 内部会乘实际采样周期。 */
    float integral; /* 已乘 Ki 的积分输出，与该环输出同单位。依次对应 CH39～42。 */
    float previous; /* 上一拍的反馈值，用于计算测量值的变化率。 */
    float derivative; /* 低通后的反馈变化率，单位为反馈单位/s。微分项为 -Kd × derivative。 */
    bool primed; /* 已保存首个反馈样本，可在下一拍开始计算变化率。 */
} pid_tune_t;

/* 一次 PWM 同步采样交给控制器的数据。errors 中的位按 enum trip 解释。 */
typedef struct
{
    float phase[3]; /* A/B/C 三相电流，单位 A，从桥输出流向电机为正。对应 CH15～17。 */
    float bus; /* 功率桥的直流母线电压，单位 V，用于保护和占空比换算。对应 CH8。 */
    uint16_t encoder; /* AS5048A 单圈原始码，0～16383，跨整圈时回绕。对应 CH9。 */
    unsigned errors; /* 本拍采样、驱动和时序检查得到的异常位。 */
} sample_t;

/* 同一次对齐相对正扫起点的有符号编码器位移，以跨圈累计码计算。 */
typedef struct
{
    int sweep, before_end, end; /* 正扫终点、结束前 100 ms、回程终点。 */
} align_trace_t;

typedef struct
{
    enum mode mode;
    unsigned faults; /* axis_trip 锁存的故障位，复位清零。 */
    unsigned elapsed; /* 本次出力经过的控制拍数，由空闲开始运动时清零，用于对齐和点动计时。 */
    unsigned lease; /* 需要续期的模式距上次有效给定或 keep 的拍数，5000 拍触发通信超时。 */
    unsigned tick; /* 总控制拍数，空闲时也递增，用于外环分频和波形时间戳。 */
    unsigned command_result; /* 最近命令处理结果，对应 CH6。 */
    bool aligned; /* 本次对齐已取得有效方向和零点，允许进入闭环模式。 */
    bool hold_rpm; /* 单发 rpm 后保持台架速度给定，停机或故障时清零。 */
    bool encoder_seen; /* 已有可作差分基准的角度样本，首帧只建立基准。 */
    int direction; /* +1 或 -1。乘到编码器位移上，使正机械方向与正向旋转磁场一致。 */
    uint16_t last_count; /* 上一帧单圈原始码，用于识别 16383 与 0 之间的跨界。 */
    int64_t counts; /* 跨圈累加的编码器位移，单位为计数，保留编码器原始方向。 */
    int64_t origin; /* 对齐回程结束时的 counts，用作机械位置及电角度的零点。 */
    int64_t align_start; /* 正扫前的 counts，与扫动终点和回程终点比较。 */
    align_trace_t align_trace;
    int64_t speed_counts[RPM_WINDOW_MS]; /* 历史累计计数，跨过编码器单圈零点仍可直接相减。 */
    unsigned speed_slot; /* 下一次写入的位置，也是窗口中最早的采样位置。 */
    float position; /* 相对对齐零点的机械角，单位 rad，可跨多圈。CH12。 */
    float speed; /* 低通后的机械角速度，单位 rad/s，1 kHz 更新。CH13。 */
    float angle; /* Park 变换使用的电角度，单位 rad，范围 [-π, π)。CH14。 */
    float target; /* 上位机原始给定，电流、速度、位置模式分别用 A、rad/s、rad。CH47。 */
    float speed_ref; /* 速度环给定，单位 rad/s，来自串口速度命令或位置环输出。CH25。 */
    float iq_ref; /* q 轴转矩电流给定，单位 A。比较 CH20 和 CH21 可看电流跟踪。 */
    float id, iq; /* d 轴磁链方向、q 轴转矩方向的反馈电流，单位 A。CH19、CH21。 */
    float vd, vq; /* 交给调制器的 d/q 轴电压，单位 V。闭环时来自电流 PID。CH22、CH23。 */
    float duty[3]; /* A/B/C 三相高侧占空比，0～1，送入 TIMER0 比较寄存器。CH26～28。 */
    pid_tune_t pid[4]; /* 由 loop_id 索引，依次保存 d 电流、q 电流、速度和位置环的增益与历史量。 */
    bool cog_enabled; /* 用户开关。仅对齐通过、方向匹配的 rpm 模式施加 D 表前馈。 */
    float cog_current; /* 本次速度环实际叠加的转矩电流，单位 A。 */
    float previous_id, previous_iq; /* 上一拍未滤波的反馈，单位 A。 */
    float deadtime_duty[3]; /* 1 kHz 更新的三相平均占空比修正，与速度环错开计算。 */
} axis_t;

bool axis_cog_ready(const axis_t* a);
void axis_init(axis_t* a);
void axis_trip(axis_t* a, unsigned reason);
void axis_command(axis_t* a, const command_t* c, bool ready);
void axis_step(axis_t* a, const sample_t* s);
bool command_parse(char* line, command_t* c);
bool encoder_frame_ok(uint16_t word);
uint16_t spi_even_parity(uint16_t word, uint16_t parity_bit);
#endif
