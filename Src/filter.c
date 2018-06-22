#include "filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

void Filter_Average(FilterAverage_t *avg)
{
    if(avg->counter <= avg->sample_start){
        // do nothing until samples has stabilized
        avg->counter++;
    }
    if((avg->counter > avg->sample_start) && (avg->counter < avg->sample_stop)){
        avg->total += (avg->sample);
        avg->counter++;
        avg->average = avg->total /(avg->counter - avg->sample_start);
        //Serial.print(offset);
        //Serial.print(", ");
    }
    if(avg->counter >= avg->sample_stop){
        avg->ready = true; 
    }
}

void Filter_Lowpass(FilterLowpass_t *lp)
{
    const float tau = 1.0f/(2.0f*M_PI*lp->cf); // Time constant
    const float alpha = lp->dt/(tau + lp->dt); // See (10) in http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
    
    // y(n) = y(n-1) + alpha*(u(n) - y(n-1))
    lp->output = lp->output_last + alpha*(lp->input - lp->output_last);
    lp->output_last = lp->output;
}

float Filter_Transition(float data1, float data2, float damping)
{
    return (data1 * (1.0f - damping) + data2 * damping);
}