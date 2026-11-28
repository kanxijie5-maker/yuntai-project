#ifndef GIMBAL_DECIMAL_H
#define GIMBAL_DECIMAL_H
#include <stdbool.h>
#include <stdint.h>

/* 串口只接收普通十进制定点文本，最多六位小数。整数运算解析后一次换算，避免在 MCU 上使用堆。 */
static inline bool decimal_value(const char* p, float* value)
{
    bool negative = *p == '-';
    if(negative || *p == '+')
    {
        ++p;
    }
    uint32_t whole = 0, fraction = 0, scale = 1;
    bool seen = false;
    while(*p >= '0' && *p <= '9')
    {
        seen = true;
        whole = whole * 10 + (unsigned)(*p++ - '0');
        if(whole > 2000)
        {
            return false;
        }
    }
    if(*p == '.')
    {
        ++p;
        while(*p >= '0' && *p <= '9')
        {
            if(scale == 1000000u)
            {
                return false;
            }
            seen = true;
            fraction = fraction * 10 + (unsigned)(*p++ - '0');
            scale *= 10;
        }
    }
    if(!seen || *p)
    {
        return false;
    }
    float result = (float)whole + (float)fraction / (float)scale;
    *value = negative ? -result : result;
    return true;
}

#endif
