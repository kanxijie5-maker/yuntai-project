/* Arm Compiler 5 使用 C99。条件不成立时数组长度为负，仍在编译期报错。 */
#define AXIS_JOIN_INNER(a, b) a##b
#define AXIS_JOIN(a, b) AXIS_JOIN_INNER(a, b)
#define _Static_assert(condition, message) extern char AXIS_JOIN(axis_static_check_, __LINE__)[(condition) ? 1 : -1]
