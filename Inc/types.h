#ifndef __TYPES_H__
#define __TYPES_H__

// Low level struct types
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

/*typedef struct{
    float pressure;
    float temperature;
    float altitude;
} baro_data_t;*/



// Sensor level struct types
typedef struct{
    ypr_data_t angle;
    xyz_data_t gyro;
    float dt;
} em7180_attitude_data_t;

typedef struct{
    float altitude; // baro
    float acc_z;
    float dt;       // dt acc
    float dt_baro;  // dt baro
} em7180_altitude_data_t;

typedef struct{
    xyz_data_t acc;
    float dt;
} em7180_heading_data_t;

typedef struct{
    float altitude;
    float dt;
} ms5803_altitude_data_t;

typedef struct{
    float range;
    float dt;
} vl53l0x_range_data_t;

// Task level struct types
//
/*typedef struct{
    ypr_data_t angle;
    xyz_data_t gyro;
    float dt;
} attitude_data_t;*/

typedef struct{
    float altitude;
    float velocity;
    float dt;
} altitude_data_t;

typedef struct{
    float x; // north?
    float y; // west?
    float yaw; // ?
    float rate_x;
    float rate_y;
    float dt;
} heading_data_t;

/*
typedef struct{
    float altitude;
    xyz_data_t acc_z;
    uint32_t dt;
} altitude_data_t;

typedef struct{
    ypr_data_t angle;
    xyz_data_t gyro;
    uint32_t dt;
} heading_data_t;

typedef struct{
    float altitude;
    float rate;
} altitude_hold_data_t;
*/

#endif