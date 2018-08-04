#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <math.h>
#include "i2c1_threadsafe.h"

#include "hardware.h"
#include "uart_print.h"
#include "bmp180.h"

uint32_t BMP180_ReadRawPressure(void);
uint16_t BMP180_ReadRawTemperature(void);
uint16_t BMP180_Read16(uint8_t reg);
uint8_t BMP180_Read8(uint8_t reg);
int32_t BMP180_ComputeB5(int32_t UT);
void BMP180_Write8(uint8_t reg, uint8_t value);

// Output Queue
extern osMailQId mailBMP180ToAltHandle;

uint8_t oversampling;

int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;


bool BMP180_Init(void) 
{
    oversampling = BMP180_ULTRAHIGHRES;
    
    if (BMP180_Read8(0xD0) != 0x55){
        return false;
    }
    
    /* read calibration data */
    ac1 = BMP180_Read16(BMP180_CAL_AC1);
    ac2 = BMP180_Read16(BMP180_CAL_AC2);
    ac3 = BMP180_Read16(BMP180_CAL_AC3);
    ac4 = BMP180_Read16(BMP180_CAL_AC4);
    ac5 = BMP180_Read16(BMP180_CAL_AC5);
    ac6 = BMP180_Read16(BMP180_CAL_AC6);
    
    b1 = BMP180_Read16(BMP180_CAL_B1);
    b2 = BMP180_Read16(BMP180_CAL_B2);
    
    mb = BMP180_Read16(BMP180_CAL_MB);
    mc = BMP180_Read16(BMP180_CAL_MC);
    md = BMP180_Read16(BMP180_CAL_MD);
    
    return true;
}

int32_t BMP180_ComputeB5(int32_t UT) 
{
    int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
    int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
    return X1 + X2;
}

uint16_t BMP180_ReadRawTemperature(void) 
{
    BMP180_Write8(BMP180_CONTROL, BMP180_READTEMPCMD);
    osDelay(5);
    
    return BMP180_Read16(BMP180_TEMPDATA);
}

uint32_t BMP180_ReadRawPressure(void) 
{
    uint32_t raw;
    
    BMP180_Write8(BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));
    
    switch(oversampling)
	{ 
    case BMP180_ULTRALOWPOWER   : osDelay(5); break; 
    case BMP180_STANDARD        : osDelay(8); break; 
    case BMP180_HIGHRES         : osDelay(14); break; 
    case BMP180_ULTRAHIGHRES    : osDelay(26); break; 
	}
    
    raw = BMP180_Read16(BMP180_PRESSUREDATA);
    
    raw <<= 8;
    raw |= BMP180_Read8(BMP180_PRESSUREDATA+2);
    raw >>= (8 - oversampling);
    
    return raw;
}


int32_t BMP180_ReadPressure(void)
{
    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;
    
    UT = BMP180_ReadRawTemperature();
    UP = BMP180_ReadRawPressure();
    
    B5 = BMP180_ComputeB5(UT);
    
    // do pressure calcs
    B6 = B5 - 4000;
    X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
    X2 = ((int32_t)ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;
    
    X1 = ((int32_t)ac3 * B6) >> 13;
    X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );
    
    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    
    p = p + ((X1 + X2 + (int32_t)3791)>>4);
    
    return p;
}

float BMP180_ReadTemperature(void) 
{
    int32_t UT, B5;     // following ds convention
    float temp;
    
    UT = BMP180_ReadRawTemperature();
    
    B5 = BMP180_ComputeB5(UT);
    temp = (B5+8) >> 4;
    temp /= 10;
    
    return temp;
}

uint8_t BMP180_Read8(uint8_t reg)
{
    uint8_t value;
    
    //I2C1_TS_Mem_Read((uint16_t)BMP180_ADDRESS << 1, reg, 1, &value, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)BMP180_ADDRESS << 1, reg, 1, &value, 1, 1000);
    
    return value;
}

uint16_t BMP180_Read16(uint8_t reg)
{
    uint8_t value[2];
    
    //I2C1_TS_Mem_Read((uint16_t)BMP180_ADDRESS << 1, reg, 1, value, 2, 1000);
    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)BMP180_ADDRESS << 1, reg, 1, value, 2, 1000);
    
    return (value[0] << 8) | value[1];
}

void BMP180_Write8(uint8_t reg, uint8_t value) 
{
    uint8_t data_send[2];
    data_send[0] = reg;
    data_send[1] = value;
    
    //I2C1_TS_Master_Transmit((uint16_t)BMP180_ADDRESS << 1, data_send, 2, 1000);
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)BMP180_ADDRESS << 1, data_send, 2, 1000);
}


void BMP180_StartTask(void const * argument)
{    
    uint32_t wakeTime = osKernelSysTick();
    uint32_t lastTime = 0;
    
    static Bmp180Altitude_t *pBmp180Altitude;
    pBmp180Altitude = osMailAlloc(mailBMP180ToAltHandle, osWaitForever);
    
    
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
        float pressure = BMP180_ReadPressure(); // in Si units for Pascal
        pressure /= 100;        
        float altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));
        //UART_Print(" %.4f\n\r", altitude);
        // Assign pointer
        pBmp180Altitude->altitude = altitude;
        pBmp180Altitude->dt = (float)dt * 0.001;
        // Send data by mail to altitude task
        osMailPut(mailBMP180ToAltHandle, pBmp180Altitude);
        //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}
