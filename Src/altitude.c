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

#include "vl53l0x.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

// Input mail
extern osMailQId myMailEM7180ToAltHandle;
//extern osMailQId myMailMS5803ToAltHandle;
// Output mail
extern osMailQId myMailAltToQuadHandle;


filter_average_t avg_acc = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

filter_average_t avg_baro = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

/*filter_average_t avg_baro2 = {
.sample_start = 0,
.sample_stop = 100,
.ready = false
};*/

filter_lowpass_t lowpass_acc_acc = {
    .cf = 5.0f
};

filter_lowpass_t lowpass_baro_alt = {
    .cf = 1.0f
};

filter_lowpass_t lowpass_baro_vel = {
    .cf = 5.0f
};

altitude_data_t altitude_data = {
    .velocity = 0,
    .altitude = 0,
    .dt = 0
};

// TODO: Most (all?) of these can be local when i don't need to print them anymore

uint8_t readyy = 0;

float acc_raw;
float baro_raw;
float dt;
float dt_baro;
float dt_tot = 0;

// Sensor offset
float acc_offset = 0;
float baro_offset = 0;

// Input data without offset
float baro_altitude = 0; 
float acc_acceleration = 0;
// LP filtered input signals
float acc_acceleration_lp;
float baro_altitude_lp;
// Derived velocity
float baro_velocity;
float baro_velocity_lp;
// Before complementary
float temp_altitude = 0;
float temp_velocity = 0;
// Finished out data
float altitude = 0;
float velocity = 0;

// Dummy out data
float acc_altitude = 0;
float acc_vel = 0;

void AltitudeStartTask(void const * argument)
{    
    //VL53L0X_init();
    // Mail from EM7180 task
    em7180_altitude_data_t *em7180_ptr; //em7180_ptr
    osEvent EM7180Event;
    // Mail from MS5803 task
    //ms5803_altitude_data_t *ms5803_ptr;
    //osEvent MS5803Event;
    
	while(1){
        EM7180Event = osMailGet(myMailEM7180ToAltHandle, osWaitForever);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        if (EM7180Event.status == osEventMail) {
            em7180_ptr = EM7180Event.value.p;
            
            baro_raw = em7180_ptr->altitude;
            acc_raw = em7180_ptr->acc_z;
            dt = em7180_ptr->dt;
            dt_baro = em7180_ptr->dt_baro;
            
            osMailFree(myMailEM7180ToAltHandle, em7180_ptr);
            
            // Calculate accelerometer offset on the z-axis
            if(!avg_acc.ready){
                avg_acc.sample = acc_raw;
                filter_average(&avg_acc);
                acc_offset = avg_acc.average;
            }
            acc_acceleration = acc_raw - acc_offset;
            // Calculate barometer1 offset
            if(!avg_baro.ready){
                avg_baro.sample = baro_raw;
                filter_average(&avg_baro);
                baro_offset = avg_baro.average;
            }
            baro_altitude = baro_raw - baro_offset;
            
            if(avg_acc.ready && avg_baro.ready){
                
                if(readyy == 0){
                    //UART_Print("=========================");
                    UART_Print(" bo: %.4f", baro_offset);
                    UART_Print(" ao: %.4f\n\r", acc_offset);
                    //UART_Print(" dt_baro: %.8f", dt_baro);
                    //UART_Print(" dt: %.8f\n\r", dt);
                    readyy = 1;
                }
                // Low pass filter barometric altitude
                lowpass_baro_alt.input = 100 * baro_altitude;
                lowpass_baro_alt.dt = dt_baro;
                filter_lowpass(&lowpass_baro_alt);
                baro_altitude_lp = lowpass_baro_alt.output;
                
                // Derive altitude and get velocity in cm/s
                static float baro_altitude_last;
                baro_velocity = (baro_altitude_lp - baro_altitude_last) / dt_baro;
                baro_altitude_last = baro_altitude_lp;
                
                // Low pass filter barometric velocity signal
                lowpass_baro_vel.input = baro_velocity;
                lowpass_baro_vel.dt = dt_baro;
                filter_lowpass(&lowpass_baro_vel);
                baro_velocity_lp = lowpass_baro_vel.output;
                
                // Low pass filter accelerometer acceleration
                lowpass_acc_acc.input = acc_acceleration;
                lowpass_acc_acc.dt = dt;
                filter_lowpass(&lowpass_acc_acc);
                acc_acceleration_lp = lowpass_acc_acc.output;
                
                //--------------------
                static float acc_vel_last;
                acc_vel = acc_vel_last + acc_acceleration_lp * dt;
                acc_vel_last = acc_vel;
                
                static float acc_alt_last;
                acc_altitude = acc_alt_last + acc_vel * dt + 0.5f * acc_acceleration_lp * dt * dt;
                acc_alt_last = acc_altitude;
                //--------------------
                
                // Velocity = initial velocity +  (acceleration * time)
                temp_velocity = velocity + (acc_acceleration_lp * dt);
                
                // Position = initial position + (initial velocity * time) + (0.5 * acceleration * (time^2))
                temp_altitude = altitude + (temp_velocity * dt) + (0.5 * acc_acceleration_lp * dt * dt);
                
                // Calculate velocity using complimentary filter
                static float vel_coeff = 0.999;
                velocity = vel_coeff * temp_velocity + (1 - vel_coeff) * baro_velocity_lp; // <---------------
                // Calculate altitude with complimentary filter
                static float alt_coeff = 0.995;
                altitude = alt_coeff * temp_altitude + (1 - alt_coeff) * baro_altitude_lp;
                
                altitude_data.velocity = velocity;
                altitude_data.altitude = altitude;
                altitude_data.dt = dt;
            }
        }
        /*MS5803Event = osMailGet(myMailMS5803ToAltHandle, 0);
        if (MS5803Event.status == osEventMail) {
        ms5803_ptr = MS5803Event.value.p;
        
        baro2 = ms5803_ptr->altitude;
        
        osMailFree(myMailMS5803ToAltHandle, ms5803_ptr);
    }*/
        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
}


static uint32_t counter = 0;
void TelemetryStartTask2(void const * argument)
{
    osDelay(8000);
	while(1){
        osDelay(100); // TODO: osDelayUntil
        if(counter < 1000){
            dt_tot += dt;
            counter++;
            //UART_Print("Total %d", xPortGetMinimumEverFreeHeapSize());
            //UART_Print(" pitch: %.4f", pitch);
            //UART_Print(" roll: %.4f", roll);
            //UART_Print(" yaw: %.4f", yaw);
            //UART_Print(" gx: %.4f", gyro_x);
            //UART_Print(" gy: %.4f", gyro_y);
            //UART_Print(" gz: %.4f", gyro_z);
            //UART_Print(" b: %.4f", baro_raw);
            //UART_Print(" b2: %.4f", baro2);
            UART_Print(" %.4f", dt_tot);
            UART_Print(" %.4f", acc_altitude);
            UART_Print(" %.4f", 100 * baro_altitude);
            UART_Print(" %.4f", altitude);
            
            //UART_Print(" bo: %.4f", baro_offset);
            //UART_Print(" ao: %.4f", acc_z_offset);
            //UART_Print(" dt: %.4f", dt);
            //UART_Print(" alt: %.4f", altitude_data.altitude);
            //UART_Print(" vel: %.4f", altitude_data.velocity);
            UART_Print("\n\r");
        }
    }
}