#ifndef FILTER_H
#define FILTER_H

#include <stdbool.h>
#include <stdint.h>

/* Kalman IMU init example for angle and rate:

FilterKalman_t angleFilterKalman = {
    .F[0][0] = 1,
    .F[0][1] = -dt,
    .F[1][0] = 0,
    .F[1][1] = 1,

    .B[0] = dt,
    .B[1] = 0,

    .Q[0] = 0.001,  // Variance for gyro
    .Q[1] = 0.003,  // Variance for gyro bias
    .R[0] = 0.03,   // Variance for accelerometer
    .R[1] = 0.03,   // Variance for accelerometer

    .H[0][0] = 1,
    .H[0][1] = 0,
    .H[1][0] = 0,
    .H[1][1] = 0,
    
    .P[0][0] = 0,
    .P[0][1] = 0,
    .P[1][0] = 0,
    .P[1][1] = 0
};

angleFilterKalman.u = gyro_value;    // The raw gyro 
angleFilterKalman.z[0] = acc_value;  // The raw accelerometer value
angleFilterKalman.z[1] = 0;

Filter_Kalman(&angleFilterKalman);

float angle = angleFilterKalman.x[0];
float rate = angleFilterKalman.u - angleFilterKalman.x[1];

*/

typedef struct{
    // Inputs
    float u;        // Control input
    float z[2];     // Measurement input (raw baro data (IMU raw acc data))
    float dt;       // Timestamp
    // Output matrix
    float x[2];     // Output
    // Initialize variance for each sensor
    float Q[2];     // Process (control) variance (for u)
    float R[2];     // Measurement variance (for z)
    // Initialize models
    float H[2][2];  // Observation model
    float F[2][2];  // State transition model
    float B[2];     // Control input model
    float P[2][2];  // Covariance matrix
    // Don't initialize
    float y[2];
    float S[2][2];
    float K[2][2];
} FilterKalman_t;

typedef struct{
    uint32_t sample_start;
    uint32_t sample_stop;
    uint32_t counter;
    float sample;
    float total;
    float average;
    bool ready;
} FilterAverage_t;

typedef struct{
    float input;
    float output_last;
    float output;
    float cf; //cut-off frequency (higher value lower filtering)
    float dt;
} FilterLowpass_t;

//float Filter_Kalman(float acc, float gyro, float dt);
void Filter_Kalman(FilterKalman_t *k);
void Filter_Average(FilterAverage_t *filter); // Filter_Average(filterAverage_t *avg)
void Filter_Lowpass(FilterLowpass_t *filter); // Filter_Lowpass(filtlerLowpass_t *lp)
float Filter_Transition(float data1, float data2, float damping); // Filter_Transition(

#endif