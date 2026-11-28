/* R2.4 的三相零偏累计。每轴独立清零一个 current_zero_t，再开始一次校准。 */
#ifndef CURRENT_ZERO_H
#define CURRENT_ZERO_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t sum[3]; /* 256 组全部通过后，各项除以 256 即该相零偏，单位为 ADC 码。 */
    unsigned count;
} current_zero_t;

/* eligible 由板级代码判定：三相高阻、PWM 主输出关闭、供电和驱动读回正常。
 * 按 R2.4 的 12 位 ADC 与半参考窗口检查；无效样本会打断连续计数。成功后由调用方保存结果。 */
static inline bool current_zero_sample(current_zero_t* z, const uint16_t adc[3], uint16_t mid, bool eligible)
{
    bool good = eligible && mid >= 1750 && mid <= 2350;
    for(unsigned i = 0; i < 3; ++i)
    {
        int delta = (int)adc[i] - mid;
        good = good && delta > -70 && delta < 70;
    }
    if(!good)
    {
        *z = (current_zero_t){0};
        return false;
    }
    if(z->count < 256u)
    {
        for(unsigned i = 0; i < 3; ++i)
            z->sum[i] += adc[i];
        ++z->count;
    }
    return z->count == 256u;
}
#endif
