#ifndef UTILS_H
#error Dont include this file directly, include utils.h instead
#endif

template<typename T, typename R>
bool in_between(T min_val, T val, T max_val)
{
    return (min_val <= val) && (val <= max_val);
}

template<typename T>
int individual_size(T cont)
{
    return sizeof(cont) / sizeof(cont[0]);
}