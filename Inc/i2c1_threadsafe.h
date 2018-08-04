#ifndef I2C_THREADSAFE_H
#define I2C_THREADSAFE_H

void I2C1_TS_Mem_Read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
void I2C1_TS_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif