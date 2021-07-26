/* Util functions */
#ifndef UTILS_H
#define UTILS_H
#include <math.h>
 
// Bool to string
const char * const bool_to_string(bool b);

// Normalize angle between -pi and pi
float normalize_angle(float angle);

#endif
