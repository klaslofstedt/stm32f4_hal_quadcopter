#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdbool.h>
#include <stdint.h>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

inline float mapf(float val, float in_min, float in_max, float out_min, float out_max);
inline uint32_t map(uint32_t val, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);


#endif