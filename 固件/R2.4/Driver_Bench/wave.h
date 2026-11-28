#ifndef AXIS_WAVE_H
#define AXIS_WAVE_H
#include <stdint.h>
#include <string.h>

enum
{
    CHANNELS = 64
};

typedef struct
{
    float ch[CHANNELS]; /* CH0～63 按 main.c 中 pack 的顺序排列，每个通道占 4 字节。 */
    uint32_t tail; /* 帧尾在串口线上依次为 00 00 80 7F，上位机据此划分波形帧。 */
} wave_frame_t;

_Static_assert(sizeof(wave_frame_t) == (CHANNELS + 1) * 4, "JustFloat frame size");

/* GD32 为小端 IEEE 754，直接复制通道；正无穷的字节序列只用作 VOFA+ 帧尾。 */
static inline void wave_pack(wave_frame_t* frame, const float* channels)
{
    memcpy(frame->ch, channels, sizeof(frame->ch));
    frame->tail = 0x7f800000u;
}
#endif
