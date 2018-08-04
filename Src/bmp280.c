#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <math.h>

#include "hardware.h"
#include "uart_print.h"
#include "types.h"
#include "bmp280.h"

#define BMP280_ADDRESS                      0x76
#define BMP280_CHIPID                       0x58


#define BMP280_REGISTER_DIG_T1              0x88
#define BMP280_REGISTER_DIG_T2              0x8A
#define BMP280_REGISTER_DIG_T3              0x8C

#define BMP280_REGISTER_DIG_P1              0x8E
#define BMP280_REGISTER_DIG_P2              0x90
#define BMP280_REGISTER_DIG_P3              0x92
#define BMP280_REGISTER_DIG_P4              0x94
#define BMP280_REGISTER_DIG_P5              0x96
#define BMP280_REGISTER_DIG_P6              0x98
#define BMP280_REGISTER_DIG_P7              0x9A
#define BMP280_REGISTER_DIG_P8              0x9C
#define BMP280_REGISTER_DIG_P9              0x9E

#define BMP280_REGISTER_CHIPID              0xD0
#define BMP280_REGISTER_VERSION             0xD1
#define BMP280_REGISTER_SOFTRESET           0xE0

#define BMP280_REGISTER_CAL26               0xE1  // R calibration stored in 0xE1-0xF0

#define BMP280_REGISTER_CONTROL             0xF4
#define BMP280_REGISTER_CONFIG              0xF5
#define BMP280_REGISTER_PRESSUREDATA        0xF7
#define BMP280_REGISTER_TEMPDATA            0xFA

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bmp280_calib_data;

void BMP280_readCoefficients(void);
void BMP280_write8(uint8_t reg, uint8_t value);
uint8_t BMP280_read8(uint8_t reg);
uint16_t BMP280_read16(uint8_t reg);
uint32_t BMP280_read24(uint8_t reg);
int16_t BMP280_readS16(uint8_t reg);
uint16_t BMP280_read16_LE(uint8_t reg); // little endian
int16_t BMP280_readS16_LE(uint8_t reg); // little endian 


int32_t t_fine;

bmp280_calib_data _bmp280_calib;

// Output Queue
extern osMailQId mailBMP280ToAltHandle;

bool BMP280_Init(void)
{
    if (BMP280_read8(BMP280_REGISTER_CHIPID) != BMP280_CHIPID){
        return false;
    }
    
    BMP280_readCoefficients();
    BMP280_write8(BMP280_REGISTER_CONTROL, 0x3F);
    return true;
}


void BMP280_write8(uint8_t reg, uint8_t value) 
{
    uint8_t data_send[2];
    data_send[0] = reg;
    data_send[1] = value;

    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)BMP280_ADDRESS << 1, data_send, 2, 1000);
}


uint8_t BMP280_read8(uint8_t reg)
{
    uint8_t value;

    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)BMP280_ADDRESS << 1, reg, 1, &value, 1, 1000);

    return value;
}


uint16_t BMP280_read16(uint8_t reg)
{
    uint8_t value[2];
    
    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)BMP280_ADDRESS << 1, reg, 1, value, 2, 1000);

    return (value[0] << 8) | value[1]; // TODO: change order?
}

uint16_t BMP280_read16_LE(uint8_t reg)
{
    uint16_t temp = BMP280_read16(reg);
    return (temp >> 8) | (temp << 8);
    
}


int16_t BMP280_readS16(uint8_t reg)
{
    return (int16_t)BMP280_read16(reg);
    
}

int16_t BMP280_readS16_LE(uint8_t reg)
{
    return (int16_t)BMP280_read16_LE(reg);
    
}


uint32_t BMP280_read24(uint8_t reg)
{
    uint8_t temp[3];
    
    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)BMP280_ADDRESS << 1, reg, 1, temp, 3, 1000);

    uint32_t value;
    
    value = temp[0]; // TODO: change order to 2 1 0?
    value <<= 8;
    value |= temp[1];
    value <<= 8;
    value |= temp[2];
    
    return value;
}

void BMP280_readCoefficients(void)
{
    _bmp280_calib.dig_T1 = BMP280_read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = BMP280_readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = BMP280_readS16_LE(BMP280_REGISTER_DIG_T3);
    
    _bmp280_calib.dig_P1 = BMP280_read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = BMP280_readS16_LE(BMP280_REGISTER_DIG_P9);
}


float BMP280_readTemperature(void)
{
    int32_t var1, var2;
    
    int32_t adc_T = BMP280_read24(BMP280_REGISTER_TEMPDATA);
    adc_T >>= 4;
    
    var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
             ((int32_t)_bmp280_calib.dig_T2)) >> 11;
    
    var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
               ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
             ((int32_t)_bmp280_calib.dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    
    float T  = (t_fine * 5 + 128) >> 8;
    return T/100;
}


float BMP280_readPressure(void)
{
    int64_t var1, var2, p;
    
    // Must be done first to get the t_fine variable set up
    BMP280_readTemperature();
    
    int32_t adc_P = BMP280_read24(BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
        ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;
    
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
    return (float)p/256;
}

float BMP280_readAltitude(float seaLevelhPa)
{
    seaLevelhPa = 1013.25;
    
    float altitude;
    
    float pressure = BMP280_readPressure(); // in Si units for Pascal
    pressure /= 100;
    
    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
    
    return altitude;
}

void BMP280_StartTask(void const * argument)
{    
    uint32_t wakeTime = osKernelSysTick();
    uint32_t lastTime = 0;
    
    static Bmp280Altitude_t *pBmp280Altitude;
    pBmp280Altitude = osMailAlloc(mailBMP280ToAltHandle, osWaitForever);
    
    
	while(1){
        osDelayUntil(&wakeTime, 50); //20
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
        
        wakeTime = osKernelSysTick();
        uint32_t dt = wakeTime - lastTime;
        lastTime = wakeTime;
        
        //float temperature, pressure;
        // Read barometer data
        //MS5803_Read(&pressure, &temperature, MS5803_D1_4096, MS5803_D2_4096);
        //float altitude = (float)(44307.7 * (1.0 - (pow(pressure / 1013.25, 0.190284))));
        //UART_Print(" %.4f\n\r", altitude);
        // Calculate dt
        float pressure = BMP280_readPressure(); // in Si units for Pascal
        pressure /= 100;
    
        float altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));
        //UART_Print(" %.4f\n\r", altitude);
        // Assign pointer
        pBmp280Altitude->altitude = altitude;
        pBmp280Altitude->dt = (float)dt * 0.001;
        // Send data by mail to altitude task
        osMailPut(mailBMP280ToAltHandle, pBmp280Altitude);
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}