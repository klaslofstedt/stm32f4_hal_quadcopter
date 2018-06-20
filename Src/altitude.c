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

// This gives a 50cm transition window from vl53l0x to barometer
// Accelerometer is used at all times
// No vl53l0x min, no baro max
#define MAX_VL53L0X_ALT 150 // 1.5m
#define MIN_BARO_ALT 100    // 1m

// Input mail
extern osMailQId myMailEM7180ToAltHandle;
extern osMailQId myMailMS5803ToAltHandle;
extern osMailQId myMailVL53L0XToAltHandle;
// Output mail
extern osMailQId myMailAltToQuadHandle;


filter_average_t avg_acc = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

filter_average_t avg_baro1 = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

filter_average_t avg_baro2 = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

filter_average_t avg_range = {
    .sample_start = 500,
    .sample_stop = 600,
    .counter = 0,
    .ready = false
};

filter_lowpass_t lowpass_acc_acc = {
    .cf = 5.0f
};

filter_lowpass_t lowpass_baro1_alt = {
    .cf = 1.0f
};

filter_lowpass_t lowpass_baro2_alt = {
    .cf = 1.0f
};

filter_lowpass_t lowpass_baro1_vel = {
    .cf = 5.0f
};

filter_lowpass_t lowpass_sensors_vel = {
    .cf = 5.0f
};

/*altitude_data_t altitude_data = {
    .velocity = 0,
    .altitude = 0,
    .dt = 0
};*/

// TODO: Most (all?) of these can be local when i don't need to print them anymore

uint8_t readyy = 0;

static float acc_raw;
static float range_raw;
static float baro1_raw;
static float baro2_raw;

static float acc_dt;
static float baro1_dt;
static float baro2_dt;
static float dt_tot;
static float range_dt;
static float sensors_dt;


// Sensor offset
static float acc_offset;
static float baro1_offset;
static float baro2_offset;
static float range_offset;

// Input data without offset
static float baro1_altitude; 
static float baro2_altitude;
static float acc_acceleration;
static float range_altitude;

// Brometers and vl53l0x (and other altitude measuring senors?) added together
static float sensors_altitude;
 
// LP filtered input signals
static float acc_acceleration_lp;
static float baro1_altitude_lp;
static float baro2_altitude_lp;
static float baro_tot_altitude;
//static float range_lp;
// Derived velocity 
static float baro1_velocity;
static float baro1_velocity_lp;
static float sensors_velocity;
static float sensors_velocity_lp;
//static float baro2_velocity;
//static float baro2_velocity_lp;

// Before complementary
static float temp_altitude = 0;
static float temp_velocity = 0;
// 
static float temp_altitude2 = 0;
static float temp_velocity2 = 0;

// Finished out data
static float altitude = 0;
static float velocity = 0;
//
static float altitude2 = 0;
static float velocity2 = 0;

// Dummy out data
static float acc_altitude = 0;
static float acc_vel = 0;

void AltitudeStartTask(void const * argument)
{    
    // Mail from EM7180 task
    em7180_altitude_data_t *em7180_alt_ptr; //em7180_ptr
    osEvent EM7180Event;
    // Mail from MS5803 task
    ms5803_altitude_data_t *ms5803_alt_ptr;
    osEvent MS5803Event;
    // Mail from VL53l0X task
    vl53l0x_range_data_t *vl53l0x_range_ptr;
    osEvent VL53L0XEvent;
    // Mail to quadcopter task
    altitude_data_t *alt_quad_ptr;
    alt_quad_ptr = osMailAlloc(myMailAltToQuadHandle, osWaitForever);
    
	while(1){
        // Wait forever for acc & baro data from the em7180 task
        EM7180Event = osMailGet(myMailEM7180ToAltHandle, osWaitForever);
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        if (EM7180Event.status == osEventMail) {
            em7180_alt_ptr = EM7180Event.value.p;
            
            baro1_raw = em7180_alt_ptr->altitude;
            acc_raw = em7180_alt_ptr->acc_z;
            acc_dt = em7180_alt_ptr->dt;
            baro1_dt = em7180_alt_ptr->dt_baro;
            
            osMailFree(myMailEM7180ToAltHandle, em7180_alt_ptr);
        }
        // Poll data from baro data from the ms5803 task
        MS5803Event = osMailGet(myMailMS5803ToAltHandle, 0);
        if (MS5803Event.status == osEventMail) {
            ms5803_alt_ptr = MS5803Event.value.p;
            
            baro2_raw = ms5803_alt_ptr->altitude;
            baro2_dt = ms5803_alt_ptr->dt;
            
            osMailFree(myMailMS5803ToAltHandle, ms5803_alt_ptr);
        }
        // Poll data from range data from the vl53l0x task
        VL53L0XEvent = osMailGet(myMailVL53L0XToAltHandle, 0);
        if (VL53L0XEvent.status == osEventMail) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            vl53l0x_range_ptr = VL53L0XEvent.value.p;
            
            range_raw = vl53l0x_range_ptr->range;
            range_dt = vl53l0x_range_ptr->dt;
            
            osMailFree(myMailVL53L0XToAltHandle, vl53l0x_range_ptr);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        }
        
        // Calculate accelerometer offset on the z-axis
        if(!avg_acc.ready){
            avg_acc.sample = acc_raw;
            filter_average(&avg_acc);
            acc_offset = avg_acc.average;
        }
        acc_acceleration = acc_raw - acc_offset;
        // Calculate barometer1 offset
        if(!avg_baro1.ready){
            avg_baro1.sample = baro1_raw;
            filter_average(&avg_baro1);
            baro1_offset = avg_baro1.average;
        }
        baro1_altitude = baro1_raw - baro1_offset;
        
        // Calculate barometer1 offset
        if(!avg_baro2.ready){
            avg_baro2.sample = baro2_raw;
            filter_average(&avg_baro2);
            baro2_offset = avg_baro2.average;
        }
        baro2_altitude = baro2_raw - baro2_offset;
        
        // Calculate range offset
        if(!avg_range.ready){
            avg_range.sample = range_raw;
            filter_average(&avg_range);
            range_offset = avg_range.average;
        }
        range_altitude = range_raw - range_offset;
        
        if(avg_acc.ready && avg_baro1.ready && avg_baro2.ready){
            
            if(readyy == 0){
                //UART_Print("=========================");
                UART_Print(" ao: %.4f\n\r", acc_offset);
                UART_Print(" bo: %.4f", baro1_offset);
                UART_Print(" bo: %.4f", baro2_offset);
                UART_Print(" ro: %.4f", range_offset);
                UART_Print("\n\r");
                //UART_Print(" baro1_dt: %.8f", baro1_dt);
                //UART_Print(" acc_dt: %.8f\n\r", acc_dt);
                readyy = 1;
            }
            // Low pass filter barometric altitude
            lowpass_baro1_alt.input = 100 * baro1_altitude;
            lowpass_baro1_alt.dt = baro1_dt;
            filter_lowpass(&lowpass_baro1_alt);
            baro1_altitude_lp = lowpass_baro1_alt.output;
            
            lowpass_baro2_alt.input = 100 * baro2_altitude;
            lowpass_baro2_alt.dt = baro2_dt;
            filter_lowpass(&lowpass_baro2_alt);
            baro2_altitude_lp = lowpass_baro2_alt.output;
            
            // TODO: This method does not take into consideration that we might 
            // wanna land somewhere higher/lower than the starting point, so
            // basically this is a pretty shit method without some sort of 
            // offset reset, but will do now for hover tests
            
            // The vl53l0x doesn't need lowpass-filtering, so let's add upp all
            // altitude measuring sensors before it's fused together with
            // accelerometer data
            baro_tot_altitude = (baro1_altitude_lp + baro1_altitude_lp) / 2; // TODO: change one baro1 to working baro2
            // If vl53l0x is within range but barometer out of range
            if((range_altitude <= MAX_VL53L0X_ALT) && (baro_tot_altitude < MIN_BARO_ALT)){
                sensors_altitude = range_altitude;
                sensors_dt = range_dt;
            }
            // If vl53l0x is out of range but barometer within range
            if((baro_tot_altitude >= MIN_BARO_ALT) && (range_altitude > MAX_VL53L0X_ALT)){
                sensors_altitude = range_altitude;
                sensors_dt = baro1_dt;
            }
            // If vl53l0x and barometer within range, combine them in a 50cm transition
            if((baro_tot_altitude >= MIN_BARO_ALT) && (range_altitude <= MAX_VL53L0X_ALT)){
                // Calculate a damping value between 0 and 1 (0 - 50 / 50)
                float damping = (float)((sensors_altitude - MIN_BARO_ALT) / (MAX_VL53L0X_ALT - MIN_BARO_ALT));
                // The closer to 0, the bigger the contribution of range, the closer to 1, the bigger the contribution from baro
                sensors_altitude = filter_transition(range_altitude, baro_tot_altitude, damping);
                sensors_dt = range_dt;
            }
                
            // Derive altitude and get velocity in cm/s
            static float baro1_altitude_last;
            baro1_velocity = (baro1_altitude_lp - baro1_altitude_last) / baro1_dt;
            baro1_altitude_last = baro1_altitude_lp;
            
            // Derive sensors altitude and get velocity in cm/s
            // TODO: find a better way to determine sensors_dt, since baro is
            // sampled in 50Hz and vl53l0x in 25Hz
            static float sensors_altitude_last;
            sensors_velocity = (sensors_altitude - sensors_altitude_last) / sensors_dt;
            sensors_altitude_last = sensors_altitude;
            
            lowpass_sensors_vel.input = sensors_velocity;
            lowpass_sensors_vel.dt = sensors_dt;
            filter_lowpass(&lowpass_sensors_vel);
            sensors_velocity_lp = lowpass_sensors_vel.output;
            
            // Low pass filter barometric velocity signal
            lowpass_baro1_vel.input = baro1_velocity;
            lowpass_baro1_vel.dt = baro1_dt;
            filter_lowpass(&lowpass_baro1_vel);
            baro1_velocity_lp = lowpass_baro1_vel.output;
            
            // Low pass filter accelerometer acceleration
            lowpass_acc_acc.input = acc_acceleration;
            lowpass_acc_acc.dt = acc_dt;
            filter_lowpass(&lowpass_acc_acc);
            acc_acceleration_lp = lowpass_acc_acc.output;
            
            //-------------------- Only for show
            static float acc_vel_last;
            acc_vel = acc_vel_last + acc_acceleration_lp * acc_dt;
            acc_vel_last = acc_vel;
            
            static float acc_alt_last;
            acc_altitude = acc_alt_last + acc_vel * acc_dt + 0.5f * acc_acceleration_lp * acc_dt * acc_dt;
            acc_alt_last = acc_altitude;
            //--------------------
            
            // Velocity = initial velocity +  (acceleration * time)
            temp_velocity = velocity + (acc_acceleration_lp * acc_dt); 
            // Position = initial position + (initial velocity * time) + (0.5 * acceleration * (time^2))
            temp_altitude = altitude + (temp_velocity * acc_dt) + (0.5 * acc_acceleration_lp * acc_dt * acc_dt);
            // Calculate velocity using complimentary filter 
            static float vel_coeff = 0.999;
            velocity = vel_coeff * temp_velocity + (1 - vel_coeff) * baro1_velocity_lp;
            // Calculate altitude with complimentary filter
            static float alt_coeff = 0.995;
            altitude = alt_coeff * temp_altitude + (1 - alt_coeff) * baro1_altitude_lp;
            
            // TODO: This block uses the combined laser and barometer data and can
            // probably be used instead of the block above as soon as it's
            // verified :) ---------------------
            // Velocity = initial velocity +  (acceleration * time)
            temp_velocity2 = velocity2 + (acc_acceleration_lp * acc_dt); 
            // Position = initial position + (initial velocity * time) + (0.5 * acceleration * (time^2))
            temp_altitude2 = altitude2 + (temp_velocity2 * acc_dt) + (0.5 * acc_acceleration_lp * acc_dt * acc_dt);
            // Calculate velocity using complimentary filter 
            static float vel2_coeff = 0.99;
            velocity2 = vel2_coeff * temp_velocity2 + (1 - vel2_coeff) * sensors_velocity_lp;
            // Calculate altitude with complimentary filter
            static float alt2_coeff = 0.95;
            altitude2 = alt2_coeff * temp_altitude2 + (1 - alt2_coeff) * sensors_altitude;
            //------------------------------
        }
        // TODO: assign data with laser
        alt_quad_ptr->velocity = velocity;
        alt_quad_ptr->altitude = altitude;
        alt_quad_ptr->dt = acc_dt;
        //
        osMailPut(myMailAltToQuadHandle, alt_quad_ptr);
        /*MS5803Event = osMailGet(myMailMS5803ToAltHandle, 0);
        if (MS5803Event.status == osEventMail) {
        ms5803_ptr = MS5803Event.value.p;
        
        baro2 = ms5803_ptr->altitude;
        
        osMailFree(myMailMS5803ToAltHandle, ms5803_ptr);
    }*/
        
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
}


static uint32_t counter = 0;
void TelemetryStartTask2(void const * argument)
{
    osDelay(6000);
	while(1){
        osDelay(40); // TODO: osDelayUntil
        if(counter < 250){
            dt_tot += acc_dt;
            counter++;
            //UART_Print("Total %d", xPortGetMinimumEverFreeHeapSize());
            //UART_Print(" pitch: %.4f", pitch);
            //UART_Print(" roll: %.4f", roll);
            //UART_Print(" yaw: %.4f", yaw);
            //UART_Print(" gx: %.4f", gyro_x);
            //UART_Print(" gy: %.4f", gyro_y);
            //UART_Print(" gz: %.4f", gyro_z);
            //UART_Print(" b: %.4f", baro1_raw);
            //UART_Print(" b2: %.4f", baro2);
            
            //----- Print to plot
            //UART_Print(" %.4f", dt_tot);
            //UART_Print(" %.4f", acc_altitude);
            //UART_Print(" %.4f", 100 * baro1_altitude);
            //UART_Print(" %.4f", altitude);
            //------
            
            //----- Print to plot
            //UART_Print(" %.4f", dt_tot);
            //UART_Print(" %.4f", baro1_altitude_lp);
            //UART_Print(" %.4f", baro2_altitude_lp);
            //UART_Print(" %.4f", baro_tot_altitude);
            //------
            
            //----- Print to plot
            UART_Print(" %.4f", dt_tot);
            UART_Print(" %.4f", acc_altitude);
            UART_Print(" %.4f", range_altitude);
            UART_Print(" %.4f", altitude2);
            //------
            
            //UART_Print(" %.4f", dt_tot);
            //UART_Print(" %.4f", range_raw);
            
            //UART_Print(" ms5803 %.4f", baro2_altitude);
            
            //UART_Print(" bo: %.4f", baro1_offset);
            //UART_Print(" ao: %.4f", acc_z_offset);
            //UART_Print(" dt: %.4f", dt);
            //UART_Print(" alt: %.4f", altitude_data.altitude);
            //UART_Print(" vel: %.4f", altitude_data.velocity);
            UART_Print("\n\r");
        }
    }
}