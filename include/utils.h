/* Util functions */
#ifndef UTILS_H
#define UTILS_H
#include <math.h>
 
const char * const bool_to_string(bool b);

// Normalize angle between -pi and pi
float normalize_angle(float angle);

// Returns individual size of container
int individual_size(float* cont);
template<typename T>
int individual_size(T cont);

// Check if val is between min_val and max_val, like min_val <= val <= max_val
bool in_between(float min_val, float val, float max_val);
template<typename T, typename R>
bool in_between(T min_val, T val, T max_val);

#include "utils.tcc"
#endif
