#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct{
    uint32_t sample_start;
    uint32_t sample_stop;
    uint32_t counter;
    float sample;
    float total;
    float average;
    bool ready;
} filter_average_t;

void filter_average(filter_average_t *avg);

#endif