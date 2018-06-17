#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"
#include "stm32f4xx_hal.h"

#include "hardware.h"

//#define I2C_DEBUG

int VL53L0X_i2c_init(void) {
  //Wire.begin();
  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
  /*Wire.beginTransmission(deviceAddress);
  Wire.write(index);

  while(count--) {
    Wire.write((uint8_t)pdata[0]);

    pdata++;
  }

  Wire.endTransmission();*/
  
  
    uint8_t data_index[1];
    data_index[0] = index;
    
    //xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)deviceAddress << 1, data_index, 1, 1000);
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)deviceAddress << 1, pdata, count, 1000);
    
    return VL53L0X_ERROR_NONE;
}

int VL53L0X_read_multi(uint8_t deviceAddress, uint8_t index, uint8_t *pdata, uint32_t count) {
  /*Wire.beginTransmission(deviceAddress);
  Wire.write(index);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (byte)count);


  while (count--) {
    pdata[0] = Wire.read();

    pdata++;
  }*/
   HAL_I2C_Mem_Read(&hi2c1, (uint16_t)deviceAddress << 1, index, 1, pdata, count, 1000);

  return VL53L0X_ERROR_NONE;
}

int VL53L0X_write_byte(uint8_t deviceAddress, uint8_t index, uint8_t data) {
    uint8_t buff[2];
    buff[0] = index;
    buff[1] = data;
    
    //xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)deviceAddress << 1, buff, 2, 1000);
    return VL53L0X_ERROR_NONE;//VL53L0X_write_multi(deviceAddress, index, &data, 1);
}

int VL53L0X_write_word(uint8_t deviceAddress, uint8_t index, uint16_t data) {
  uint8_t buff[3];
  buff[2] = data & 0xFF;
  buff[1] = data >> 8;
  buff[0] = index;

    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)deviceAddress << 1, buff, 3, 1000);
  
  return VL53L0X_ERROR_NONE;//VL53L0X_write_multi(deviceAddress, index, buff, 2);
}

int VL53L0X_write_dword(uint8_t deviceAddress, uint8_t index, uint32_t data) {
  uint8_t buff[5];

  buff[4] = data & 0xFF;
  buff[3] = data >> 8;
  buff[2] = data >> 16;
  buff[1] = data >> 24;
  buff[0] = index;
  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)deviceAddress << 1, buff, 5, 1000);

  return VL53L0X_ERROR_NONE;//VL53L0X_write_multi(deviceAddress, index, buff, 4);
}

int VL53L0X_read_byte(uint8_t deviceAddress, uint8_t index, uint8_t *data) {
    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)deviceAddress << 1, index, 1, data, 1, 1000);
  return VL53L0X_ERROR_NONE;//VL53L0X_read_multi(deviceAddress, index, data, 1);
}

int VL53L0X_read_word(uint8_t deviceAddress, uint8_t index, uint16_t *data) {
  uint8_t buff[2];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 2);

  uint16_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  *data = tmp;

  return r;
}

int VL53L0X_read_dword(uint8_t deviceAddress, uint8_t index, uint32_t *data) {
  uint8_t buff[4];
  int r = VL53L0X_read_multi(deviceAddress, index, buff, 4);

  uint32_t tmp;
  tmp = buff[0];
  tmp <<= 8;
  tmp |= buff[1];
  tmp <<= 8;
  tmp |= buff[2];
  tmp <<= 8;
  tmp |= buff[3];

  *data = tmp;

  return r;
}
