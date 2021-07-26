#include "utils.h"
// Bool to string
const char * const bool_to_string(bool b){ return b ? "true" : "false"; }

// Normalize angle between -pi and pi
float normalize_angle(float angle){
    if(angle > M_PI) angle = angle - 2 * M_PI;
    else if(angle < - M_PI) angle = angle + 2 * M_PI;
    return angle;
}