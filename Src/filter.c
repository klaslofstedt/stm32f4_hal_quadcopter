#include "filter.h"

void filter_average(filter_average_t *avg)
{
    if(avg->counter < avg->sample_start){
        // do nothing until samples has stabilized
        avg->counter++;
    }
    if((avg->counter > avg->sample_start-1) && (avg->counter < avg->sample_stop)){
        avg->total += (avg->sample);
        avg->counter++;
        avg->average = avg->total /(avg->counter - avg->sample_start);
        //Serial.print(offset);
        //Serial.print(", ");
    }
    if(avg->counter == avg->sample_stop){
        avg->ready = true; 
    }
}