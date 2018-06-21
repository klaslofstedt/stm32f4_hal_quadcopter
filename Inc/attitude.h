#ifndef ATTITUDE_H
#define ATTITUDE_H

typedef struct{
    float x; // north?
    float y; // west?
    float yaw; // ?
    float rate_x;
    float rate_y;
    float dt;
} heading_data_t;

/*typedef struct{
    Ypr_t quaternions;
    Ypr_t gyro;
    uint32_t dt;
    UBaseType_t stack_size;
} attitude_data_t;*/

//void AttitudeTask(void *pvParameters);


#endif