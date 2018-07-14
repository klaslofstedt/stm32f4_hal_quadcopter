#include "filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif


void Filter_Kalman(FilterKalman_t *k)
{
    // x = F*x + B*u
    float x_temp[2];
    x_temp[0] = k->F[0][0] * k->x[0] + k->F[0][1] * k->x[1] + k->B[0]*k->u; // Position
    x_temp[1] = k->F[1][0] * k->x[0] + k->F[1][1] * k->x[1] + k->B[1]*k->u; // Velocity
    k->x[0] = x_temp[0];
    k->x[1] = x_temp[1];
    //rate = u - x[1];

    // P = F * P * Ft + Q
    float P_temp[2][2]; // Why is Q multiplied with dt?
    P_temp[0][0] = k->F[0][0] * (k->F[0][0]*k->P[0][0]+k->F[0][1]*k->P[1][0]) + k->F[0][1] * (k->F[0][0]*k->P[0][1]+k->F[0][1]*k->P[1][1]) + k->Q[0]*k->dt;
    P_temp[0][1] = k->F[1][0] * (k->F[0][0]*k->P[0][0]+k->F[0][1]*k->P[1][0]) + k->F[1][1] * (k->F[0][0]*k->P[0][1]+k->F[0][1]*k->P[1][1]);
    P_temp[1][0] = k->F[0][0] * (k->F[1][0]*k->P[0][0]+k->F[1][1]*k->P[1][0]) + k->F[0][1] * (k->F[1][0]*k->P[0][1]+k->F[1][1]*k->P[1][1]);
    P_temp[1][1] = k->F[1][0] * (k->F[1][0]*k->P[0][0]+k->F[1][1]*k->P[1][0]) + k->F[1][1] * (k->F[1][0]*k->P[0][1]+k->F[1][1]*k->P[1][1]) + k->Q[1]*k->dt;

    k->P[0][0] = P_temp[0][0];
    k->P[0][1] = P_temp[0][1];
    k->P[1][0] = P_temp[1][0];
    k->P[1][1] = P_temp[1][1];

    // S = H*P * Ht + R
    k->S[0][0] = k->H[0][0] * k->H[0][0] * k->P[0][0] + k->R[0];
    k->S[0][1] = k->H[0][1] * k->H[1][0] * k->P[0][1];
    k->S[1][0] = k->H[0][1] * k->H[1][0] * k->P[1][0];
    k->S[1][1] = k->H[1][1] * k->H[1][1] * k->P[1][1] + k->R[1];

    // K = P * Ht * S^-1
    float S_inv = k->S[0][0]*k->S[1][1] - k->S[0][1]*k->S[1][0];

    k->K[0][0] = ((k->H[0][0]*k->P[0][0] + k->H[0][1]*k->P[0][1])*k->S[1][1] - (k->H[1][0]*k->P[0][0] + k->H[1][1]*k->P[0][1])*k->S[1][0]) / S_inv;
    k->K[0][1] = ((k->H[1][0]*k->P[0][0] + k->H[1][1]*k->P[0][1])*k->S[0][0] - (k->H[0][0]*k->P[0][0] + k->H[0][1]*k->P[0][1])*k->S[0][1]) / S_inv;
    k->K[1][0] = ((k->H[0][0]*k->P[1][0] + k->H[0][1]*k->P[1][1])*k->S[1][1] - (k->H[1][0]*k->P[1][0] + k->H[1][1]*k->P[1][1])*k->S[1][0]) / S_inv;
    k->K[1][1] = ((k->H[1][0]*k->P[1][0] + k->H[1][1]*k->P[1][1])*k->S[0][0] - (k->H[0][0]*k->P[1][0] + k->H[0][1]*k->P[1][1])*k->S[0][1]) / S_inv;

    // y = z - (H*x)
    k->y[0] = k->z[0] - (k->H[0][0]*k->x[0] + k->H[0][1]*k->x[1]);
    k->y[1] = k->z[1] - (k->H[1][0]*k->x[0] + k->H[1][1]*k->x[1]);

    // x = x + K*y
    k->x[0] = k->x[0] + k->K[0][0]*k->y[0] + k->K[0][1]*k->y[1];
    k->x[1] = k->x[1] + k->K[1][0]*k->y[0] + k->K[1][1]*k->y[1];

    // P = (I -K*H)*P
    P_temp[0][0] = k->P[0][0]*(1 - k->H[0][0]*k->K[0][0] - k->H[1][0]*k->K[0][1]);
    P_temp[0][1] = k->P[0][1]*(0 - k->H[0][1]*k->K[0][0] - k->H[1][1]*k->K[0][1]);
    P_temp[1][0] = k->P[1][0]*(0 - k->H[0][0]*k->K[1][0] - k->H[1][0]*k->K[1][1]);
    P_temp[1][1] = k->P[1][1]*(1 - k->H[0][1]*k->K[1][0] - k->H[1][1]*k->K[1][1]);

    k->P[0][0] = P_temp[0][0];
    k->P[0][1] = P_temp[0][1];
    k->P[1][0] = P_temp[1][0];
    k->P[1][1] = P_temp[1][1];
}



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