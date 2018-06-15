#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "task.h"

#include "math.h"

#include "hardware.h"
#include "em7180.h"
#include "ms5803.h"
#include "uart_print.h"
#include "altitude.h"
#include "types.h"
#include "filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

// Input mail
extern osMailQId myMailEM7180ToAltHandle;
extern osMailQId myMailMS5803ToAltHandle;
// Output mail
extern osMailQId myMailAltToQuadHandle;


filter_average_t avg_acc = {
  .sample_start = 0,
  .sample_stop = 100,
  .ready = false
};

filter_average_t avg_baro1 = {
  .sample_start = 0,
  .sample_stop = 100,
  .ready = false
};

filter_average_t avg_baro2 = {
  .sample_start = 0,
  .sample_stop = 100,
  .ready = false
};

static float acc_z_offset = 0;
static float baro1_offset = 0;
static float baro2_offset = 0;

float baro1, baro2;
float acc;

void AltitudeStartTask(void const * argument)
{    


    altitude_data_t  *altitude_ptr;
    osEvent EM7180Event;
    
	while(1){
        EM7180Event = osMailGet(myMailEM7180ToAltHandle, osWaitForever);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        if (EM7180Event.status == osEventMail) {
            altitude_ptr = EM7180Event.value.p;
            
            baro1 = altitude_ptr->altitude;
            acc = altitude_ptr->acc_z;
            
            osMailFree(myMailEM7180ToAltHandle, altitude_ptr);
            
            // Calculate accelerometer offset on the z-axis
            if(!avg_acc.ready){
                avg_acc.sample = acc;
                filter_average(&avg_acc);
                acc_z_offset = avg_acc.average;
            }
            // Calculate barometer1 offset
            if(!avg_baro1.ready){
                avg_baro1.sample = baro1;
                filter_average(&avg_baro1);
                baro1_offset = avg_baro1.average;
            }
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        }
    }
}

void TelemetryStartTask2(void const * argument)
{
	while(1){
        osDelay(200); // TODO: osDelayUntil
        //UART_Print("Total %d", xPortGetMinimumEverFreeHeapSize());
        //UART_Print(" pitch: %.4f", pitch);
        //UART_Print(" roll: %.4f", roll);
        //UART_Print(" yaw: %.4f", yaw);
        //UART_Print(" gx: %.4f", gyro_x);
        //UART_Print(" gy: %.4f", gyro_y);
        //UART_Print(" gz: %.4f", gyro_z);
        UART_Print(" b: %.4f", baro1);
        UART_Print(" a: %.4f", acc);
        UART_Print(" bo: %.4f", baro1_offset);
        UART_Print(" ao: %.4f", acc_z_offset);
        //UART_Print(" a3: %.4f", a3);
        UART_Print("\n\r");
    }
}