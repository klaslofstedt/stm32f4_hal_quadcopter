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

typedef struct{
    float input;
    float output_last;
    float output;
    float cf; //cut-off frequency (higher value lower filtering)
    float dt;
} filter_lowpass_t;


void filter_average(filter_average_t *temp);
void filter_lowpass(filter_lowpass_t *temp);
float filter_transition(float data1, float data2, float damping);

#endif