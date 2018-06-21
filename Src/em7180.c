/* 

EM7180.cpp: Class implementation for EM7180 SENtral Sensor

Adapted from

https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/EM7180_MPU9250_BMP280

This file is part of EM7180.

EM7180 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

EM7180 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "math.h"

#include "hardware.h"
#include "em7180.h"
#include "uart_print.h"

#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42

// EM7180 SENtral register map
// see http://www.emdeveloper.com/downloads/7180/EMSentral_EM7180_Register_Map_v1_3.pdf
//
#define EM7180_QX                 0x00  // this is a 32-bit normalized floating point number read from registers 0x00-03
#define EM7180_QY                 0x04  // this is a 32-bit normalized floating point number read from registers 0x04-07
#define EM7180_QZ                 0x08  // this is a 32-bit normalized floating point number read from registers 0x08-0B
#define EM7180_QW                 0x0C  // this is a 32-bit normalized floating point number read from registers 0x0C-0F
#define EM7180_QTIME              0x10  // this is a 16-bit unsigned integer read from registers 0x10-11
#define EM7180_MX                 0x12  // int16_t from registers 0x12-13
#define EM7180_MY                 0x14  // int16_t from registers 0x14-15
#define EM7180_MZ                 0x16  // int16_t from registers 0x16-17
#define EM7180_MTIME              0x18  // uint16_t from registers 0x18-19
#define EM7180_AX                 0x1A  // int16_t from registers 0x1A-1B
#define EM7180_AY                 0x1C  // int16_t from registers 0x1C-1D
#define EM7180_AZ                 0x1E  // int16_t from registers 0x1E-1F
#define EM7180_ATIME              0x20  // uint16_t from registers 0x20-21
#define EM7180_GX                 0x22  // int16_t from registers 0x22-23
#define EM7180_GY                 0x24  // int16_t from registers 0x24-25
#define EM7180_GZ                 0x26  // int16_t from registers 0x26-27
#define EM7180_GTIME              0x28  // uint16_t from registers 0x28-29
#define EM7180_Baro               0x2A  // start of two-byte MS5637 pressure data, 16-bit signed interger
#define EM7180_BaroTIME           0x2C  // start of two-byte MS5637 pressure timestamp, 16-bit unsigned
#define EM7180_Temp               0x2E  // start of two-byte MS5637 temperature data, 16-bit signed interger
#define EM7180_TempTIME           0x30  // start of two-byte MS5637 temperature timestamp, 16-bit unsigned
#define EM7180_QRateDivisor       0x32  // uint8_t 
#define EM7180_EnableEvents       0x33
#define EM7180_HostControl        0x34
#define EM7180_EventStatus        0x35
#define EM7180_SensorStatus       0x36
#define EM7180_SentralStatus      0x37
#define EM7180_AlgorithmStatus    0x38
#define EM7180_FeatureFlags       0x39
#define EM7180_ParamAcknowledge   0x3A
#define EM7180_SavedParamByte0    0x3B
#define EM7180_SavedParamByte1    0x3C
#define EM7180_SavedParamByte2    0x3D
#define EM7180_SavedParamByte3    0x3E
#define EM7180_ActualMagRate      0x45
#define EM7180_ActualAccelRate    0x46
#define EM7180_ActualGyroRate     0x47
#define EM7180_ActualBaroRate     0x48
#define EM7180_ActualTempRate     0x49
#define EM7180_ErrorRegister      0x50
#define EM7180_AlgorithmControl   0x54
#define EM7180_MagRate            0x55
#define EM7180_AccelRate          0x56
#define EM7180_GyroRate           0x57
#define EM7180_BaroRate           0x58
#define EM7180_TempRate           0x59
#define EM7180_LoadParamByte0     0x60
#define EM7180_LoadParamByte1     0x61
#define EM7180_LoadParamByte2     0x62
#define EM7180_LoadParamByte3     0x63
#define EM7180_ParamRequest       0x64
#define EM7180_ROMVersion1        0x70
#define EM7180_ROMVersion2        0x71
#define EM7180_RAMVersion1        0x72
#define EM7180_RAMVersion2        0x73
#define EM7180_ProductID          0x90
#define EM7180_RevisionID         0x91
#define EM7180_RunStatus          0x92
#define EM7180_UploadAddress      0x94 // uint16_t registers 0x94 (MSB)-5(LSB)
#define EM7180_UploadData         0x96  
#define EM7180_CRCHost            0x97  // uint32_t from registers 0x97-9A
#define EM7180_ResetRequest       0x9B   
#define EM7180_PassThruStatus     0x9E   
#define EM7180_PassThruControl    0xA0
#define EM7180_ACC_LPF_BW         0x5B  //Register GP36
#define EM7180_GYRO_LPF_BW        0x5C  //Register GP37
#define EM7180_BARO_LPF_BW        0x5D  //Register GP38

#define EM7180_ADDRESS           0x28   // Address of the EM7180 SENtral sensor hub
#define M24512DFM_DATA_ADDRESS   0x50   // Address of the 500 page M24512DRC EEPROM data buffer, 1024 bits (128 8-bit bytes) per page
#define M24512DFM_IDPAGE_ADDRESS 0x58   // Address of the single M24512DRC lockable EEPROM ID page
#define AK8963_ADDRESS           0x0C   // Address of magnetometer
#define BMP280_ADDRESS           0x76   // Address of BMP280 altimeter when ADO = 0

#ifndef M_PI
#define M_PI 3.14159265358979
#endif       

// Are these static for the EM7180?
#define ACC_SCALE   2048
#define GYRO_SCALE  65536

// Input Semaphore
extern osSemaphoreId myBinarySemEM7180InterruptHandle;
// Output Mail
extern osMailQId myMailEM7180ToQuadHandle;
extern osMailQId myMailEM7180ToAltHandle;


static uint8_t _eventStatus;

static bool AlgorithmStatus(uint8_t status);

static void SetGyroFs(uint16_t gyro_fs);
static void SetMagAccFs(uint16_t mag_fs, uint16_t acc_fs);
static void SetIntegerParam (uint8_t param, uint32_t param_val);

static float uint32_RegToFloat (uint8_t *buf);
static bool HasFeature(uint8_t features);

uint8_t errorStatus;
static void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t *dest);
static void ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);
static uint8_t ReadByte(uint8_t address, uint8_t subAddress);
static void WriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
static void ReadThreeAxis(uint8_t xreg, int16_t *x, int16_t *y, int16_t *z);


void delay(uint32_t milliseconds)
{
    osDelay(milliseconds);
}


void WriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    uint8_t data_send[2];
    data_send[0] = subAddress;
    data_send[1] = data;
    
    //xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)address << 1, data_send, 2, 1000);
    //xSemaphoreGive(xMutexI2C);
    //HAL_I2C_Mem_Write(&hi2c3, (uint16_t)address << 1, (uint16_t)subAddress, 1, /* &?*/data, 1, 1000);
}

uint8_t ReadByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data_receive;
    
    //xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    uint8_t HAL_return = HAL_I2C_Mem_Read(&hi2c3, (uint16_t)address << 1, subAddress, 1, &data_receive, 1, 1000);
    //xSemaphoreGive(xMutexI2C);
    
    return data_receive;
}

void ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *data_receive)
{  
    //xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c3, (uint16_t)address << 1, subAddress, 1, data_receive, count, 1000);
    //xSemaphoreGive(xMutexI2C);
}

float uint32_RegToFloat (uint8_t *buf)
{
    union {
        uint32_t ui32;
        float f;
    } u;
    
    u.ui32 =     (((uint32_t)buf[0]) +
                  (((uint32_t)buf[1]) <<  8) +
                      (((uint32_t)buf[2]) << 16) +
                          (((uint32_t)buf[3]) << 24));
    return u.f;
}

void SetIntegerParam(uint8_t param, uint32_t param_val) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = param_val & (0xFF);
    bytes[1] = (param_val >> 8) & (0xFF);
    bytes[2] = (param_val >> 16) & (0xFF);
    bytes[3] = (param_val >> 24) & (0xFF);
    param = param | 0x80; //Parameter is the decimal value with the MSB set high to indicate a paramter write processs
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Param LSB
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]);
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]);
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Param MSB
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, param);
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==param)) {
        STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void SetMagAccFs(uint16_t mag_fs, uint16_t acc_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = mag_fs & (0xFF);
    bytes[1] = (mag_fs >> 8) & (0xFF);
    bytes[2] = acc_fs & (0xFF);
    bytes[3] = (acc_fs >> 8) & (0xFF);
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Mag LSB
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Mag MSB
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Acc LSB
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Acc MSB
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCA); //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCA)) {
        STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

void SetGyroFs(uint16_t gyro_fs) 
{
    uint8_t bytes[4], STAT;
    bytes[0] = gyro_fs & (0xFF);
    bytes[1] = (gyro_fs >> 8) & (0xFF);
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte2, bytes[2]); //Unused
    WriteByte(EM7180_ADDRESS, EM7180_LoadParamByte3, bytes[3]); //Unused
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
    STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
    while(!(STAT==0xCB)) {
        STAT = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm
}

// I2C communication with the M24512DFM EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

//TODO: might wanna swap data_address1 & data_address2
void M24512DFMreadBytes(uint8_t address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t *data_receive)
{   
    uint16_t data_address = ((uint16_t)data_address1 << 8) | data_address2;
    HAL_I2C_Mem_Read(&hi2c3, (uint16_t)address << 1, data_address, 2, (uint8_t *)&data_receive, count, 1000);
}

bool HasFeature(uint8_t features)
{
    return features & ReadByte(EM7180_ADDRESS, EM7180_FeatureFlags);
}

bool AlgorithmStatus(uint8_t status)
{
    return ReadByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & status;
}

/*static volatile bool newData;

void interruptHandler()
{
newData = true;
}*/

// public methods ========================================================================================================

bool EM7180_EEPROM(void)
{
    errorStatus = 0;
    // Check SENtral status, make sure EEPROM upload of firmware was accomplished
    for (int attempts=0; attempts < 10; ++attempts) {
        if (ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01) {
            if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01) { }
            if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02) { }
            if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04) {
                errorStatus = 0xB0;
                return false;
            }
            if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08) { }
            if(ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10) {
                errorStatus = 0xB0;
                return false;
            }
            break;
        }
        WriteByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
        delay(500);  
    }
    
    
    if (ReadByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04) {
        errorStatus = 0xB0;
        return false;
    }
    
    return true;
}

const char *EM7180_GetErrorString(void)
{
    if (errorStatus & 0x01) return "Magnetometer error";
    if (errorStatus & 0x02) return "Accelerometer error";
    if (errorStatus & 0x04) return "Gyro error";
    if (errorStatus & 0x10) return "Magnetometer ID not recognized";
    if (errorStatus & 0x20) return "Accelerometer ID not recognized";
    if (errorStatus & 0x30) return "Math error";
    if (errorStatus & 0x40) return "Gyro ID not recognized";
    if (errorStatus & 0x80) return "Invalid sample rate";
    
    // Ad-hoc
    if (errorStatus & 0x90) return "Failed to put SENtral in pass-through mode";
    if (errorStatus & 0xA0) return "Unable to read from SENtral EEPROM";
    if (errorStatus & 0xB0) return "Unable to upload config to SENtral EEPROM";
    
    return "Unknown error";
}


bool EM7180_HasBaro(void)
{
    return HasFeature(0x01);
}

bool EM7180_HasHumidity(void)
{
    return HasFeature(0x02);
}

bool EM7180_HasTemperature(void)
{
    return HasFeature(0x04);
}

bool EM7180_HasCustom1(void)
{
    return HasFeature(0x08);
}

bool EM7180_HasCustom2(void)
{
    return HasFeature(0x10);
}

bool EM7180_HasCustom3(void)
{
    return HasFeature(0x20);
}

uint8_t EM7180_GetProductId(void) 
{
    return ReadByte(EM7180_ADDRESS, EM7180_ProductID);
}

uint8_t EM7180_GetRevisionId(void) 
{
    return ReadByte(EM7180_ADDRESS, EM7180_RevisionID);
}

uint16_t EM7180_GetRamVersion(void)
{
    uint16_t ram1 = ReadByte(EM7180_ADDRESS, EM7180_RAMVersion1);
    uint16_t ram2 = ReadByte(EM7180_ADDRESS, EM7180_RAMVersion2);
    
    return ram1 << 8 | ram2;
}

uint16_t EM7180_GetRomVersion(void)
{
    uint16_t rom1 = ReadByte(EM7180_ADDRESS, EM7180_ROMVersion1);
    uint16_t rom2 = ReadByte(EM7180_ADDRESS, EM7180_ROMVersion2);
    
    return rom1 << 8 | rom2;
}

void ReadThreeAxis(uint8_t xreg, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t rawData[6];  // x/y/z register data stored here
    ReadBytes(EM7180_ADDRESS, xreg, 6, rawData);  // Read the six raw data registers sequentially into data array
    *x = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    *y = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
    *z = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
}

bool EM7180_Passthru_Begin(void)
{
    // Do generic intialization
    if (!EM7180_EEPROM()) return false;
    
    // First put SENtral in standby mode
    uint8_t c = ReadByte(EM7180_ADDRESS, EM7180_AlgorithmControl);
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, c | 0x01);
    // Verify standby status
    // if(readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus) & 0x01) {
    // Place SENtral in pass-through mode
    WriteByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x01); 
    if(ReadByte(EM7180_ADDRESS, EM7180_PassThruStatus) & 0x01) {
    }
    else {
        errorStatus = 0x90;
        return false;
    }
    
    uint8_t data[128];
    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x00, 0x00, 128, data);
    if (data[0] != 0x2A || data[1] != 0x65) {
        errorStatus = 0xA0;
        return false;
    }
    
    // Success
    return true;
}


bool EM7180_Init()
{
    HAL_Delay(500);
    // Sensible defaults for sensor ranges
    uint8_t  aRes = 8;    // Gs
    uint16_t gRes = 2000; // radians per second
    uint16_t mRes = 1000; // microTeslas
    
    // Sensible defaults for sensor Output Data Rates (ODRs)
    uint8_t  magRate      = 100; // Hz
    uint16_t accelRate    = 250; // Hz
    uint16_t gyroRate     = 250; // Hz
    uint8_t  baroRate     = 50;  // Hz
    uint8_t  qRateDivisor = 1;   // 1/3 gyro rate
    
    // Fail immediately if unable to upload EEPROM
    if (!EM7180_EEPROM()){
        return false;
    }
    
    // Enter EM7180 initialized state
    WriteByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
    WriteByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
    WriteByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
    WriteByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
    
    // Setup LPF bandwidth (BEFORE setting ODR's)
    WriteByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03);  // 41Hz
    WriteByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 41Hz
    
    // Set accel/gyro/mage desired ODR rates
    WriteByte(EM7180_ADDRESS, EM7180_QRateDivisor, qRateDivisor-1);    
    WriteByte(EM7180_ADDRESS, EM7180_MagRate, magRate);
    WriteByte(EM7180_ADDRESS, EM7180_AccelRate, accelRate/10); 
    WriteByte(EM7180_ADDRESS, EM7180_GyroRate, gyroRate/10);   
    WriteByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | baroRate); // 0x80 = enable bit
    
    // Configure operating mode
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
    
    // Enable interrupt to host upon certain events:
    // quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    WriteByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
    
    // Enable EM7180 run mode
    WriteByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
    delay(100);
    
    // Disable stillness mode
    SetIntegerParam (0x49, 0x00);
    
    // Write desired sensor full scale ranges to the EM7180
    SetMagAccFs(mRes, aRes);
    SetGyroFs(gRes); 
    // Check event status register to clear the EM7180 interrupt before the main loop
    ReadByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt
    
    
    // If readByte() returns 0, return true
    return ReadByte(EM7180_ADDRESS, EM7180_SensorStatus) ? false : true;
}

void EM7180_GetFullScaleRanges(uint8_t *accFs, uint16_t *gyroFs, uint16_t *magFs)
{
    uint8_t param[4];
    
    // Read sensor new FS values from parameter space
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    uint8_t param_xfer = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4A)) {
        param_xfer = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    *magFs = ((uint16_t)(param[1]<<8) | param[0]);
    *accFs = ((uint8_t)(param[3]<<8) | param[2]);
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while(!(param_xfer==0x4B)) {
        param_xfer = ReadByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = ReadByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    *gyroFs = ((uint16_t)(param[1]<<8) | param[0]);
    WriteByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    WriteByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm
}

bool EM7180_AlgorithmStatusStandby(void)
{
    return AlgorithmStatus(0x01);
}

bool EM7180_AlgorithmStatusSlow(void)
{
    return AlgorithmStatus(0x02);
}

bool EM7180_AlgorithmStatusStillness(void)
{
    return AlgorithmStatus(0x04);
}

bool EM7180_AlgorithmStatusMagCalibrationCompleted(void)
{
    return AlgorithmStatus(0x08);
}

bool EM7180_AlgorithmStatusMagneticAnomalyDetected(void)
{
    return AlgorithmStatus(0x10);
}

bool EM7180_AlgorithmStatusUnreliableData(void)
{
    return AlgorithmStatus(0x20);
}

bool EM7180_RunStatusNormal()
{
    return (ReadByte(EM7180_ADDRESS, EM7180_RunStatus) & 0x01);
}

void EM7180_CheckEventStatus(void)
{
    // Check event status register, way to check data ready by checkEventStatusing rather than interrupt
    _eventStatus = ReadByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register
    
}

bool EM7180_GotError(void)
{
    if (_eventStatus & 0x02) {
        
        return true;
    }
    
    return false;
}

bool EM7180_GotQuaternion(void)
{
    return _eventStatus & 0x04;
}

bool EM7180_GotMagnetometer(void)
{
    return _eventStatus & 0x08;
}

bool EM7180_GotAccelerometer(void)
{
    return _eventStatus & 0x10;
}

bool EM7180_GotGyrometer(void)
{
    return _eventStatus & 0x20;
}

bool EM7180_GotBarometer(void)
{
    return _eventStatus & 0x40;
}

void EM7180_ReadQuaternion(float *qw, float *qx, float *qy, float *qz)
{
    uint8_t rawData[16];  // x/y/z/w quaternion register data stored here (note unusual order!)
    
    ReadBytes(EM7180_ADDRESS, EM7180_QX, 16, rawData);       
    
    *qx = uint32_RegToFloat(&rawData[0]);
    *qy = uint32_RegToFloat(&rawData[4]);
    *qz = uint32_RegToFloat(&rawData[8]);
    *qw = uint32_RegToFloat(&rawData[12]); 
}

void EM7180_ReadAccelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
    ReadThreeAxis(EM7180_AX, ax, ay, az);
}

void EM7180_ReadGyrometer(int16_t *gx, int16_t *gy, int16_t *gz)
{
    ReadThreeAxis(EM7180_GX, gx, gy, gz);
}

void EM7180_ReadMagnetometer(int16_t *mx, int16_t *my, int16_t *mz)
{
    ReadThreeAxis(EM7180_MX, mx, my, mz);
}


void EM7180_ReadBarometer(float *pressure, float *temperature)
{
    uint8_t rawData[2];
    
    ReadBytes(EM7180_ADDRESS, EM7180_Baro, 2, rawData);  // Read the two raw data registers sequentially into data array
    int16_t rawPressure =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    *pressure = (float)rawPressure *.01f + 1013.25f; // pressure in millibars
    
    // get BMP280 temperature
    ReadBytes(EM7180_ADDRESS, EM7180_Temp, 2, rawData);  // Read the two raw data registers sequentially into data array
    int16_t rawTemperature =  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
    
    *temperature = (float) rawTemperature*0.01;  // temperature in degrees C
}

uint8_t EM7180_GetActualMagRate()
{
    return ReadByte(EM7180_ADDRESS, EM7180_ActualMagRate);
}

uint16_t EM7180_GetActualAccelRate()
{
    return 10*ReadByte(EM7180_ADDRESS, EM7180_ActualAccelRate);
}

uint16_t EM7180_GetActualGyroRate()
{
    return 10*ReadByte(EM7180_ADDRESS, EM7180_ActualGyroRate);
}

uint8_t EM7180_GetActualBaroRate()
{
    return ReadByte(EM7180_ADDRESS, EM7180_ActualBaroRate);
}

uint8_t EM7180_GetActualTempRate()
{
    return ReadByte(EM7180_ADDRESS, EM7180_ActualTempRate);
}


/* StartEM7180Task function */
void EM7180StartTask(void const * argument)
{                    
    // Quaternions
    float qw, qx, qy, qz;
    // Acceleration in cm/s^2 adjusted without gravity
    float acc_x, acc_y, acc_z;
    // Gyro in m/s(?) (drift free from EM7180?)
    float gyro_x, gyro_y, gyro_z;
    // Angles in degrees
    float yaw, pitch, roll;
    // Gravitational contribution to acceleration
    float a1, a2, a3;
    //
    //bool newAltData = false;
    uint32_t wakeTime = osKernelSysTick();
    uint32_t lastTime = 0;
    
    uint32_t wakeTimeBaro = osKernelSysTick();
    uint32_t lastTimeBaro = 0;
   
	while(1){
        osSemaphoreWait(myBinarySemEM7180InterruptHandle, osWaitForever);
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        
        Em7180Attitude_t *pEm7180Attitude;
        pEm7180Attitude = osMailAlloc(myMailEM7180ToQuadHandle, osWaitForever);
        
        Em7180Altitude_t *pEm7180Altitude;
        pEm7180Altitude = osMailAlloc(myMailEM7180ToAltHandle, osWaitForever);
        
        EM7180_CheckEventStatus();
        
        if (EM7180_GotError()) {  
            UART_Print("EM7180 error: ");
            UART_Print(EM7180_GetErrorString());
        }
        
        if (EM7180_GotQuaternion()) {             
            EM7180_ReadQuaternion(&qw, &qx, &qy, &qz);
            // Calculate YPR from quaternions
            roll  = atan2(2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
            pitch = -asin(2.0f * (qx * qz - qw * qy));
            yaw   = atan2(2.0f * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);   
            // Convert from rad to deg
            roll    *= 180.0f / M_PI;
            pitch   *= 180.0f / M_PI;
            yaw     *= 180.0f / M_PI; 
            //yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
            // Ensure yaw stays between 0 and 360
            if(yaw < 0){
                yaw += 360.0f; 
            }
            // Assign the em7180-to-quadcopter pointer
            pEm7180Attitude->angle.yaw = yaw;
            pEm7180Attitude->angle.pitch = pitch;
            pEm7180Attitude->angle.roll = roll;
        }
        
        if (EM7180_GotGyrometer()) { // 250Hz
            int16_t gx, gy, gz;
            EM7180_ReadGyrometer(&gx, &gy, &gz);
            // Scale the raw gyro into ? TODO: m/s?
            gyro_x = (float)gx / GYRO_SCALE; 
            gyro_y = (float)gy / GYRO_SCALE; 
            gyro_z = (float)gz / GYRO_SCALE; 
            
            pEm7180Attitude->gyro.x = gyro_x;
            pEm7180Attitude->gyro.y = gyro_y;
            pEm7180Attitude->gyro.z = gyro_z;
        }
        
        if (EM7180_GotAccelerometer()){ // 250Hz
            int16_t ax, ay, az;
            EM7180_ReadAccelerometer(&ax, &ay, &az);
            // Scale the raw accelerometer value into G's
            acc_x = (float)ax / ACC_SCALE; 
            acc_y = (float)ay / ACC_SCALE;
            acc_z = (float)az / ACC_SCALE;
            // Calculate the gravitation contribution from quatarions
            a1 = 2.0f * (qx * qz - qw * qy);
            a2 = 2.0f * (qw * qx + qy * qz);
            a3 = qw * qw - qx * qx - qy * qy + qz * qz;
            // Remove gravitation
            acc_x -= a1;
            acc_y -= a2;
            acc_z -= a3;
            // Convert from G's into cm/s^2
            acc_x *= 9.80665f * 100.0f;
            acc_y *= 9.80665f * 100.0f;
            acc_z *= 9.80665f * 100.0f;
            //heading_data.acc_x = acc_x;
            //heading_data.acc_y = acc_y;
            pEm7180Altitude->acc_z = acc_z;
        }
        
        if(EM7180_GotBarometer()) { // 50Hz
            float temperature, pressure;
            EM7180_ReadBarometer(&pressure, &temperature);
            
            float altitude = (1.0f - powf(pressure / 1013.25f, 0.190295f)) * 44330.0f;
            // Calculate the ~50Hz dt
            wakeTimeBaro = osKernelSysTick();
            uint32_t dt_baro = wakeTimeBaro - lastTimeBaro;
            lastTimeBaro = wakeTimeBaro;
            // Assign em7180-to-altitude pointer and convert from ms to s
            pEm7180Altitude->dt_baro = (float)dt_baro*0.001;
            pEm7180Altitude->altitude = altitude;
            //newAltData = true;
        }
        
        // Calculate ~250Hz dt
        wakeTime = osKernelSysTick();
        uint32_t dt = wakeTime - lastTime;
        lastTime = wakeTime;
        // Convert from milliseconds to seconds
        pEm7180Attitude->dt = (float)dt*0.001;
        pEm7180Altitude->dt = (float)dt*0.001;
        
        // Send attitude data by mail to quadcopter task 250 Hz
        osMailPut(myMailEM7180ToQuadHandle, pEm7180Attitude);
        // Send altitude data by mail to altitude task 250 Hz
        osMailPut(myMailEM7180ToAltHandle, pEm7180Altitude);
        
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}