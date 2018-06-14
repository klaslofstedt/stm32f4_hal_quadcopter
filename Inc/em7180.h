#ifndef EM7180_H
#define EM7180_H

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "types.h"



void EM7180Task(void *pvParameters);

void EM7180_InterruptCallback();

bool EM7180_Init();
bool EM7180_EEPROM(void);
bool EM7180_Passthru_begin(void);


const char *EM7180_getErrorString(void);

uint8_t EM7180_getProductId(void); 
uint8_t EM7180_getRevisionId(void); 
uint16_t EM7180_getRamVersion(void);
uint16_t EM7180_getRomVersion(void);

bool EM7180_hasBaro(void);
bool EM7180_hasHumidity(void);
bool EM7180_hasTemperature(void);
bool EM7180_hasCustom1(void);
bool EM7180_hasCustom2(void);
bool EM7180_hasCustom3(void);

void EM7180_checkEventStatus(void);
bool EM7180_gotError(void);
bool EM7180_gotQuaternion(void);
bool EM7180_gotMagnetometer(void);
bool EM7180_gotAccelerometer(void);
bool EM7180_gotGyrometer(void);
bool EM7180_gotBarometer(void);
void EM7180_readMagnetometer(int16_t *mx, int16_t *my, int16_t *mz);
void EM7180_readAccelerometer(int16_t *ax, int16_t *ay, int16_t *az);
void EM7180_readGyrometer(int16_t *gx, int16_t *gy, int16_t *gz);
void EM7180_readQuaternion(float *qw, float *qx, float *qy, float *qz);
void EM7180_readBarometer(float *pressure, float *temperature);
uint8_t EM7180_getActualMagRate();
uint16_t EM7180_getActualAccelRate();
uint16_t EM7180_getActualGyroRate();
uint8_t EM7180_getActualBaroRate();
uint8_t EM7180_getActualTempRate();
bool EM7180_runStatusNormal(void);
bool EM7180_algorithmStatusStandby(void);
bool EM7180_algorithmStatusSlow(void);
bool EM7180_algorithmStatusStillness(void);
bool EM7180_algorithmStatusMagCalibrationCompleted(void);
bool EM7180_algorithmStatusMagneticAnomalyDetected(void);
bool EM7180_algorithmStatusUnreliableData(void);
void EM7180_getFullScaleRanges(uint8_t *accFs, uint16_t *gyroFs, uint16_t *magFs);


bool EM7180_Passthru_begin(void);

#endif