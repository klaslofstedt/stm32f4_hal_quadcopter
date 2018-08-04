#ifndef BMP180_H
#define BMP180_H

#include <stdbool.h>

#define BMP180_ADDRESS          0x77

#define BMP180_ULTRALOWPOWER    0
#define BMP180_STANDARD         1
#define BMP180_HIGHRES          2
#define BMP180_ULTRAHIGHRES     3
#define BMP180_CAL_AC1          0xAA
#define BMP180_CAL_AC2          0xAC
#define BMP180_CAL_AC3          0xAE  
#define BMP180_CAL_AC4          0xB0  
#define BMP180_CAL_AC5          0xB2  
#define BMP180_CAL_AC6          0xB4  
#define BMP180_CAL_B1           0xB6  
#define BMP180_CAL_B2           0xB8  
#define BMP180_CAL_MB           0xBA  
#define BMP180_CAL_MC           0xBC  
#define BMP180_CAL_MD           0xBE  

#define BMP180_CONTROL          0xF4 
#define BMP180_TEMPDATA         0xF6
#define BMP180_PRESSUREDATA     0xF6
#define BMP180_READTEMPCMD      0x2E
#define BMP180_READPRESSURECMD  0x34

typedef struct {
    float altitude;
    float dt;
} Bmp180Altitude_t;

void BMP180_StartTask(void const *argument);
bool BMP180_Init(void);
int32_t BMP180_ReadPressure(void);
float BMP180_ReadTemperature(void);



#endif