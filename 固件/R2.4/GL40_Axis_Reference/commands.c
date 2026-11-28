/* 可选的台架命令解析，仅在前台调用。控制中断直接使用 command_t。 */
#include "foc.h"
#include "decimal.h"
#include <string.h>

/* 就地拆分一行命令。游标由调用者持有，不占用堆或库函数的全局状态。 */
static char* next_token(char** cursor)
{
    char* start = *cursor + strspn(*cursor, " \t");
    char* end = start + strcspn(start, " \t");
    *cursor = end;
    if(*end)
    {
        *end = 0;
        ++*cursor;
    }
    return *start ? start : NULL;
}

/* 整行解析成功后才交给控制中断。拒绝非有限值、越界设定、缺参和多余参数。 */
bool command_parse(char* line, command_t* c)
{
    static const char* const names[] = {"stop", "align", "jog", "iq", "speed", "rpm", "pos", "keep", "pid",
        "cogon", "cogoff"
    };
    static const char* const loops[] = {"d", "q", "speed", "pos"};
    const char* token = next_token(&line);
    *c = (command_t){.op = STOP};
    if(!token)
    {
        return false;
    }
    const char* attached = NULL;
    if(strncmp(token, "rpm", 3) == 0 && token[3])
    {
        /* rpm10 和 rpm 10 共用数值校验，只有前者从命令名后取参数。 */
        attached = token + 3;
        token = "rpm";
    }
    else if(strcmp(token, "torque") == 0)
    {
        /* 未标定电机转矩常数，串口直接指定产生转矩的 Iq，单位 A。 */
        token = "iq";
    }
    else if(strcmp(token, "motor") == 0)
    {
        /* 电机带电自检沿用对齐的电流极性、正扫和回程验证。 */
        token = "align";
    }
    unsigned op = 0;
    while(op < sizeof(names) / sizeof(names[0]) && strcmp(token, names[op]))
    {
        ++op;
    }
    if(op == sizeof(names) / sizeof(names[0]))
    {
        return false;
    }
    c->op = (enum operation)op;
    unsigned count = op >= SET_IQ && op <= SET_POS ? 1 : 0;
    if(op == GAIN)
    {
        token = next_token(&line);
        unsigned i = 0;
        while(token && i < 4 && strcmp(token, loops[i]))
        {
            ++i;
        }
        if(!token || i == 4)
        {
            return false;
        }
        c->loop = (enum loop_id)i;
        count = 3;
    }
    for(unsigned i = 0; i < count; ++i)
    {
        token = attached ? attached : next_token(&line);
        if(!token)
        {
            return false;
        }
        float x;
        if(!decimal_value(token, &x))
        {
            return false;
        }
        if(op == SET_RPM)
            x *= TAU_F / 60.0f;
        c->value[i] = x;
    }
    return next_token(&line) == NULL && axis_command_valid(c);
}
