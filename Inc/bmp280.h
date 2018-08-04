#ifndef BMP280_H
#define BMP280_H

#include <stdbool.h>

typedef struct {
    float altitude;
    float dt;
} Bmp280Altitude_t;


void BMP280_StartTask(void const * argument);
bool BMP280_Init(void);
float BMP280_readTemperature(void);
float BMP280_readPressure(void);
float BMP280_readAltitude(float seaLevelhPa);


#endif