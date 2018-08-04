#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "i2c1_threadsafe.h"
#include "hardware.h"

extern osMutexId mutexI2C1Handle;

void I2C1_TS_Mem_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    osStatus status;
    status = osMutexWait(mutexI2C1Handle, osWaitForever);
    
    if (status == osOK) {
        HAL_I2C_Mem_Read(&hi2c1, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
        osMutexRelease (mutexI2C1Handle);
    }
    osThreadYield();
}

void I2C1_TS_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    osStatus status;
    status = osMutexWait(mutexI2C1Handle, osWaitForever);
    
    if (status == osOK) {
        HAL_I2C_Master_Transmit(&hi2c1, DevAddress, pData, Size, Timeout);
        osMutexRelease (mutexI2C1Handle);
    }
    osThreadYield();
}