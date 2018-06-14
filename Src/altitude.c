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

// Input Queues
extern xQueueHandle xQueueEM7180ToAlt; //queue_em7180_to_alt
extern xQueueHandle xQueueMS5803ToAlt;
// Output Queue
extern xQueueHandle xQueueAltToQuad;

//typedef enum average_t { IDLE, CALCULATING, READY };
//average_t acc_average = IDLE;
//average_t baro1_average = IDLE;
//average_t baro2_average = IDLE;

filter_average_t avg_acc = {
  .sample_start = 1000,
  .sample_stop = 2000,
  .ready = false
};

filter_average_t avg_baro1 = {
  .sample_start = 100,
  .sample_stop = 200,
  .ready = false
};

filter_average_t avg_baro2 = {
  .sample_start = 100,
  .sample_stop = 200,
  .ready = false
};

altitude_data_t altitude_data;

void AltitudeTask(void *pvParameters)
{
    
    //altitude_hold_data_t altitude_hold_data;
    
    //baro_data_t baro1;
    //baro_data_t baro2; // baro1 & baro2?
    
    //float baro1, baro2;
    //float acceleration;
    //bool newBaro = false;
    
    float acc_z_offset = 0;
    float baro1_offset = 0;
    float baro2_offset = 0;
    
	while(1){
        if(xQueueReceive(xQueueEM7180ToAlt, &altitude_data, portMAX_DELAY)){
            HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
            
            // Calculate accelerometer offset on the z-axis
            if(!avg_acc.ready){
                avg_acc.sample = altitude_data.acc_z;
                filter_average(&avg_acc);
                acc_z_offset = avg_acc.average;
            }
            if(!avg_baro1.ready){
                avg_baro1.sample = altitude_data.altitude;
                filter_average(&avg_baro1);
                baro1_offset = avg_baro1.average;
            }
            
            
            /*if(xQueueReceive(xQueueMS5803ToAlt, &barometer2_data, 0)){ // 50Hz
            
        }*/
            
            //altitude_hold_data.altitude = (baro1 + baro2) / 2; // Mock
            // Shitload of computations
            
            /*if(newBaro){
                newBaro = false;
                xQueueSend(xQueueAltToQuad, &altitude_data, 4 / portTICK_PERIOD_MS); // Wait for 250Hz
            }*/
            HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
        }
    }
}

void TelemetryTask2(void *pvParameters)
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
        UART_Print(" y: %.4f", altitude_data.altitude);
        UART_Print(" p: %.4f", altitude_data.acc_z);
        //UART_Print(" a1: %.4f", a1);
        //UART_Print(" a2: %.4f", a2);
        //UART_Print(" a3: %.4f", a3);
        UART_Print("\n\r");
    }
}