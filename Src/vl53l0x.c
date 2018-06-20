#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "vl53l0x.h"
#include "uart_print.h"
#include "types.h"

#define VERSION_REQUIRED_MAJOR  1
#define VERSION_REQUIRED_MINOR  0
#define VERSION_REQUIRED_BUILD  1

#define STR_HELPER( x ) #x
#define STR( x )        STR_HELPER(x)

VL53L0X_Error Status = VL53L0X_ERROR_NONE;
VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t *pMyDevice  = &MyDevice;
VL53L0X_Version_t Version;
VL53L0X_Version_t *pVersion   = &Version;
VL53L0X_DeviceInfo_t DeviceInfo;

VL53L0X_RangingMeasurementData_t vl53l0x_measurement;

// Output mail
extern osMailQId myMailVL53L0XToAltHandle;

bool VL53L0X_Init(void) 
{
    //UART_Print("Init VL53L0X\n\r");
    //int32_t   status_int;
    //int32_t   init_done = 0;
    
    uint32_t  refSpadCount;
    uint8_t   isApertureSpads;
    uint8_t   VhvSettings;
    uint8_t   PhaseCal;
    
    // Initialize Comms
    pMyDevice->I2cDevAddr      =  VL53L0X_I2C_ADDR;  // 7 bit addr
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;
    
    Status = VL53L0X_DataInit(&MyDevice);         // Data initialization
    Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
    if( Status == VL53L0X_ERROR_NONE )  {
         //UART_Print(" 1 \n\r ");
        if((DeviceInfo.ProductRevisionMinor != 1)){
            Status = VL53L0X_ERROR_NOT_SUPPORTED;
            //UART_Print(" 2 \n\r ");
        }
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_StaticInit( pMyDevice ); // Device Initialization
        //UART_Print(" 3 \n\r ");
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {       
        Status = VL53L0X_PerformRefSpadManagement( pMyDevice, &refSpadCount, &isApertureSpads ); // Device Initialization
        //UART_Print(" 4 \n\r ");
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_PerformRefCalibration( pMyDevice, &VhvSettings, &PhaseCal );           // Device Initialization
        //UART_Print(" 5 \n\r ");
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        Status = VL53L0X_SetDeviceMode( pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING );        // Setup in single ranging mode
        //UART_Print(" 6 \n\r ");
    }
    
    // Enable/Disable Sigma and Signal check
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
        //UART_Print(" 7 \n\r ");
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
        //UART_Print(" 8 \n\r ");
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
        //UART_Print(" 9 \n\r ");
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        Status = VL53L0X_SetLimitCheckValue( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
        //UART_Print(" 10 \n\r ");
    }
    
    if( Status == VL53L0X_ERROR_NONE ) {
        //UART_Print("Init VL53L0X succeded\n\r");
        return true;
    } 
    else{
        return false;
        //UART_Print("Init VL53L0X failed\n\r");
    }
}


VL53L0X_Error VL53L0X_getSingleRangingMeasurement(VL53L0X_RangingMeasurementData_t *RangingMeasurementData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    FixPoint1616_t LimitCheckCurrent;
    
    
    /*
    *  Step  4 : Test ranging mode
    */
    
    if( Status == VL53L0X_ERROR_NONE ) {
        
        //uart_printf( F( "sVL53L0X: PerformSingleRangingMeasurement" ) );
        
        Status = VL53L0X_PerformSingleRangingMeasurement( pMyDevice, RangingMeasurementData );
        
        
        VL53L0X_printRangeStatus( RangingMeasurementData );
        
        VL53L0X_GetLimitCheckCurrent( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent );
        
        /*uart_printf( F( "RANGE IGNORE THRESHOLD: " ) );
        uart_printf( (float)LimitCheckCurrent / 65536.0 );
        
        uart_printf( F( "Measured distance: " ) );
        uart_printf( RangingMeasurementData->RangeMilliMeter );*/
        
    }
    
    return Status;
}




void VL53L0X_printRangeStatus( VL53L0X_RangingMeasurementData_t* pRangingMeasurementData )
{
    char buf[ VL53L0X_MAX_STRING_LENGTH ];
    uint8_t RangeStatus;
    
    /*
    * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
    */
    
    RangeStatus = pRangingMeasurementData->RangeStatus;
    
    VL53L0X_GetRangeStatusString( RangeStatus, buf );
    
    /*uart_printf( F("Range Status: " ) );
    uart_printf( RangeStatus );
    uart_printf( F( " : " ) );
    uart_printf( buf );*/
}

VL53L0X_Error rangingTest(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData) 
{ 
    //debug = false;
    VL53L0X_getSingleRangingMeasurement(pRangingMeasurementData);
    
    return VL53L0X_ERROR_NONE;
}

void VL53L0XStartTask(void const * argument)
{    
    uint16_t vl53l0x_range_mm;
    uint32_t wakeTime = osKernelSysTick();
    uint32_t lastTime = 0;
    
    //ms5803_altitude_data_t *ms5803_altitude_ptr;
    vl53l0x_range_data_t *vl53l0x_range_ptr;
    vl53l0x_range_ptr = osMailAlloc(myMailVL53L0XToAltHandle, osWaitForever);
    
    // TODO: change to 50 (while not printing over uart)
	while(1){
        osDelayUntil(&wakeTime, 100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        
        wakeTime = osKernelSysTick();
        uint32_t dt = wakeTime - lastTime;
        lastTime = wakeTime;
        
        rangingTest(&vl53l0x_measurement);
        if (vl53l0x_measurement.RangeStatus != 4) {  // phase failures have incorrect data
            //printf2("Distance (mm): %d\n\r", vl53l0x_measurement.RangeMilliMeter);
            vl53l0x_range_mm = vl53l0x_measurement.RangeMilliMeter;
            //in->range_cm = (float)in->range_mm / 10;
        } else {
            //printf2(" out of range ");
            //in->range_cm = -1;
            vl53l0x_range_mm = -1;
        }
        // Calculate dt
        //UART_Print(" %.4f", (float)vl53l0x_range_mm/10);
        //UART_Print(" %d", vl53l0x_range_mm);
        // Assign pointer and convert from mm to cm
        vl53l0x_range_ptr->range = (float)vl53l0x_range_mm/10;
        vl53l0x_range_ptr->dt = (float)dt * 0.001;
        // Send data by mail to altitude task
        osMailPut(myMailVL53L0XToAltHandle, vl53l0x_range_ptr);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}
