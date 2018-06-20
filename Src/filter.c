#include "filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

void filter_average(filter_average_t *temp)
{
    if(temp->counter <= temp->sample_start){
        // do nothing until samples has stabilized
        temp->counter++;
    }
    if((temp->counter > temp->sample_start) && (temp->counter < temp->sample_stop)){
        temp->total += (temp->sample);
        temp->counter++;
        temp->average = temp->total /(temp->counter - temp->sample_start);
        //Serial.print(offset);
        //Serial.print(", ");
    }
    if(temp->counter >= temp->sample_stop){
        temp->ready = true; 
    }
}

void filter_lowpass(filter_lowpass_t *temp)
{
    const float tau = 1.0f/(2.0f*M_PI*temp->cf); // Time constant
    const float alpha = temp->dt/(tau + temp->dt); // See (10) in http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf

    // y(n) = y(n-1) + alpha*(u(n) - y(n-1))
    temp->output = temp->output_last + alpha*(temp->input - temp->output_last);
    temp->output_last = temp->output;
}

float filter_transition(float data1, float data2, float damping)
{
    return (data1 * (1.0f - damping) + data2 * damping);
}