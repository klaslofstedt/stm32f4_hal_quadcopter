#include "functions.h"


inline float mapf(float val, float in_min, float in_max, float out_min, float out_max) 
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline uint32_t map(uint32_t val, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
