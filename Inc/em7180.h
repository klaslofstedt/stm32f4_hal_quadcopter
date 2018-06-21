#ifndef EM7180_H
#define EM7180_H

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "types.h"

typedef struct {
    Ypr_t angle;
    Xyz_t gyro;
    float dt;
} Em7180Attitude_t;

typedef struct {
    float altitude;
    float acc_z;
    float dt;
    float dt_baro;
} Em7180Altitude_t;

typedef struct {
    Xyz_t acc;
    float dt;
} Em7180Heading_t;


void EM7180StartTask(void const * argument);

bool EM7180_Init();
bool EM7180_EEPROM(void);
bool EM7180_Passthru_begin(void);


const char *EM7180_GetErrorString(void);

uint8_t EM7180_GetProductId(void); 
uint8_t EM7180_GetRevisionId(void); 
uint16_t EM7180_GetRamVersion(void);
uint16_t EM7180_GetRomVersion(void);

bool EM7180_HasBaro(void);
bool EM7180_HasHumidity(void);
bool EM7180_HasTemperature(void);
bool EM7180_HasCustom1(void);
bool EM7180_HasCustom2(void);
bool EM7180_HasCustom3(void);

void EM7180_CheckEventStatus(void);
bool EM7180_GotError(void);
bool EM7180_GotQuaternion(void);
bool EM7180_GotMagnetometer(void);
bool EM7180_GotAccelerometer(void);
bool EM7180_GotGyrometer(void);
bool EM7180_GotBarometer(void);
void EM7180_ReadMagnetometer(int16_t *mx, int16_t *my, int16_t *mz);
void EM7180_ReadAccelerometer(int16_t *ax, int16_t *ay, int16_t *az);
void EM7180_ReadGyrometer(int16_t *gx, int16_t *gy, int16_t *gz);
void EM7180_ReadQuaternion(float *qw, float *qx, float *qy, float *qz);
void EM7180_ReadBarometer(float *pressure, float *temperature);
uint8_t EM7180_GetActualMagRate();
uint16_t EM7180_GetActualAccelRate();
uint16_t EM7180_GetActualGyroRate();
uint8_t EM7180_GetActualBaroRate();
uint8_t EM7180_GetActualTempRate();
bool EM7180_RunStatusNormal(void);
bool EM7180_AlgorithmStatusStandby(void);
bool EM7180_AlgorithmStatusSlow(void);
bool EM7180_AlgorithmStatusStillness(void);
bool EM7180_AlgorithmStatusMagCalibrationCompleted(void);
bool EM7180_AlgorithmStatusMagneticAnomalyDetected(void);
bool EM7180_AlgorithmStatusUnreliableData(void);
void EM7180_GetFullScaleRanges(uint8_t *accFs, uint16_t *gyroFs, uint16_t *magFs);


bool EM7180_Passthru_begin(void);

#endif