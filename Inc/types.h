#ifndef __TYPES_H__
#define __TYPES_H__

typedef struct{
    float yaw;
    float pitch;
	float roll;
} ypr_data_t;

typedef struct{
    float x;
    float y;
	float z;
} xyz_data_t;

typedef struct{
    float pressure;
    float temperature;
} baro_data_t;

typedef struct{
    ypr_data_t quaternions;
    xyz_data_t gyro;
    uint32_t dt;
} attitude_data_t;

typedef struct{
    float acc_z;
    float altitude;
    uint32_t dt;
} altitude_data_t;

typedef struct{
    float acc_x;
    float acc_y;
} heading_data_t;

typedef struct{
    float altitude;
    float rate;
    uint32_t dt;
    UBaseType_t stack_size;
} altitude_hold_data_t;

#endif