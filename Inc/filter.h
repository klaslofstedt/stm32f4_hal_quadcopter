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
} FilterAverage_t; // FilterAverage_t

typedef struct{
    float input;
    float output_last;
    float output;
    float cf; //cut-off frequency (higher value lower filtering)
    float dt;
} FilterLowpass_t; // filterLowpass_t

float mapf(float val, float in_min, float in_max, float out_min, float out_max);
uint32_t map(uint32_t val, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

void Filter_Average(FilterAverage_t *filter); // Filter_Average(filterAverage_t *avg)
void Filter_Lowpass(FilterLowpass_t *filter); // Filter_Lowpass(filtlerLowpass_t *lp)
float Filter_Transition(float data1, float data2, float damping); // Filter_Transition(

#endif