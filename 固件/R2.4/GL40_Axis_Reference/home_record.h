/* GD32F303CCT6 末尾两页交替保存单圈零位。反码最后写入，掉电留下的半条记录无效。 */
#ifndef HOME_RECORD_H
#define HOME_RECORD_H
#include <stdint.h>

typedef struct
{
    uint32_t value, inverse;
} home_record_t;

static inline int home_latest(const home_record_t pages[2])
{
    int latest = -1;
    for(int i = 0; i < 2; ++i)
    {
        uint32_t value = pages[i].value;
        if((value & 0x8000u) && pages[i].inverse == ~value)
        {
            if(latest < 0 || (uint16_t)((value >> 16) - (pages[latest].value >> 16)) == 1u)
                latest = i;
        }
    }
    return latest;
}
#endif
