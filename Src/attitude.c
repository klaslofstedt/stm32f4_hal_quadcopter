#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "math.h"

#include "hardware.h"
#include "em7180.h"
#include "uart_print.h"
#include "attitude.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

/*extern xSemaphoreHandle xSemaphoreEM7180AttInterrupt;
extern xQueueHandle xAttitudeQueue;

void AttitudeTask(void *pvParameters)
{
    attitude_data_t attitude_data;    

	while(1){
        if(xSemaphoreTake(xSemaphoreEM7180AttInterrupt, portMAX_DELAY)){
            HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
            EM7180_checkEventStatus();
            
            if (EM7180_gotError()) {  
                UART_Print("EM7180 error: ");
                UART_Print(EM7180_GetErrorString());
            }
            
            if (EM7180_gotQuaternion()) {
                float qw, qx, qy, qz;

                EM7180_readQuaternion(&qw, &qx, &qy, &qz);

                float roll  = atan2(2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
                float pitch = -asin(2.0f * (qx * qz - qw * qy));
                float yaw   = atan2(2.0f * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);   

                pitch *= 180.0f / M_PI;
                yaw   *= 180.0f / M_PI; 
                yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
                if(yaw < 0) yaw += 360.0f; // Ensure yaw stays between 0 and 360
                roll  *= 180.0f / M_PI;
                
                xQueueSend(xAttitudeQueue, &attitude_data, 0);
                //UART_Print("yaw: %.3f ", yaw);
                //UART_Print("pitch: %.3f ", pitch);
                //UART_Print("roll: %.3f\n\r", roll);
            }
            if(EM7180_gotBarometer()) { // 50Hz
                HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
                float temperature, pressure;
                EM7180_readBarometer(&pressure, &temperature);
                
                //baro1 = (1.0f - powf(pressure / 1013.25f, 0.190295f)) * 44330.0f;
                
                //newBaro = true;
                HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
            }
            
            if(EM7180_gotBarometer()) { // 50Hz
                float temperature, pressure;
                EM7180_readBarometer(&pressure, &temperature);
                
            }
            HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
        }
	}
}*/