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


FilterAverage_t accFilterAverage = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

FilterAverage_t baro1FilterAverage = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

FilterAverage_t baro2FilterAverage = {
    .sample_start = 500,
    .sample_stop = 1000,
    .counter = 0,
    .ready = false
};

FilterAverage_t rangeFilterAverage = {
    .sample_start = 500,
    .sample_stop = 600,
    .counter = 0,
    .ready = false
};

FilterLowpass_t accAccFilterLowpass = {
    .cf = 5.0f
};

FilterLowpass_t baro1AltFilterLowpass = {
    .cf = 1.0f
};

FilterLowpass_t baro2AltFilterLowpass = {
    .cf = 1.0f
};

FilterLowpass_t baro1VelFilterLowpass = {
    .cf = 5.0f
};

FilterLowpass_t sensorsVelFilterLowpass = {
    .cf = 5.0f
};

/*Altitude_t altitude_data = {
    .velocity = 0,
    .altitude = 0,
    .dt = 0
};*/

// TODO: Most (all?) of these can be local when i don't need to print them anymore

uint8_t readyy = 0;

static float accRaw;
static float rangeRaw;
static float baro1Raw;
static float baroRaw;

static float accDt;
static float baro1Dt;
static float baro2Dt;
static float totDt;
static float rangeDt;
static float sensorsDt;


// Sensor offset
static float accOffset;
static float baro1Offset;
static float baro2Offset;
static float rangeOffset;

// Input data without offset
static float baro1Altitude; 
static float baro2Altitude;
static float accAcceleration;
static float rangeAltitude;

// Brometers and vl53l0x (and other altitude measuring senors?) added together
static float sensorsAltitude;
 
// LP filtered input signals
static float accAccelerationLpf;
static float baro1AltitudeLpf;
static float baro2AltitudeLpf;
static float baroAltitudeTot;
//static float range_lp;
// Derived velocity 
static float baro1Velocity;
static float baro1VelocityLpf;
static float sensorsVelocity;
static float sensorsVelocityLpf;
//static float baro2_velocity;
//static float baro2_velocity_lp;

// Before complementary
static float tempAltitude = 0;
static float tempVelocity = 0;
// 
static float tempAltitude2 = 0;
static float tempVelocity2 = 0;

// Finished out data
static float altitude = 0;
static float velocity = 0;
//
static float altitude2 = 0;
static float velocity2 = 0;

// Dummy out data
static float accAltitude = 0;
static float accVel = 0;

void AltitudeStartTask(void const * argument)
{    
    // Mail from EM7180 task
    static Em7180Altitude_t *pEm7180Attitude; //pEm7180Alt
    osEvent EM7180Event;
    // Mail from MS5803 task
    static Ms5803Altitude_t *pMs5803Altitude;
    osEvent MS5803Event;
    // Mail from VL53l0X task
    static Vl53l0xRange_t *pVl53l0xRange;
    osEvent VL53L0XEvent;
    // Mail to quadcopter task
    Altitude_t *alt_quad_ptr;
    alt_quad_ptr = osMailAlloc(myMailAltToQuadHandle, osWaitForever);
    
	while(1){
        // Wait forever for acc & baro data from the em7180 task
        EM7180Event = osMailGet(myMailEM7180ToAltHandle, osWaitForever);
        // Turn on LED
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        
        if (EM7180Event.status == osEventMail) {
            pEm7180Attitude = EM7180Event.value.p;
            
            baro1Raw = pEm7180Attitude->altitude;
            accRaw = pEm7180Attitude->acc_z;
            accDt = pEm7180Attitude->dt;
            baro1Dt = pEm7180Attitude->dt_baro;
            
            osMailFree(myMailEM7180ToAltHandle, pEm7180Attitude);
        }
        // Poll data from baro data from the ms5803 task
        MS5803Event = osMailGet(myMailMS5803ToAltHandle, 0);
        if (MS5803Event.status == osEventMail) {
            pMs5803Altitude = MS5803Event.value.p;
            
            baroRaw = pMs5803Altitude->altitude;
            baro2Dt = pMs5803Altitude->dt;
            
            osMailFree(myMailMS5803ToAltHandle, pMs5803Altitude);
        }
        // Poll data from range data from the vl53l0x task
        VL53L0XEvent = osMailGet(myMailVL53L0XToAltHandle, 0);
        if (VL53L0XEvent.status == osEventMail) {
            //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
            pVl53l0xRange = VL53L0XEvent.value.p;
            
            rangeRaw = pVl53l0xRange->range;
            rangeDt = pVl53l0xRange->dt;
            
            osMailFree(myMailVL53L0XToAltHandle, pVl53l0xRange);
            //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        }
        
        // Calculate accelerometer offset on the z-axis
        if(!accFilterAverage.ready){
            accFilterAverage.sample = accRaw;
            Filter_Average(&accFilterAverage);
            accOffset = accFilterAverage.average;
        }
        accAcceleration = accRaw - accOffset;
        // Calculate barometer1 offset
        if(!baro1FilterAverage.ready){
            baro1FilterAverage.sample = baro1Raw;
            Filter_Average(&baro1FilterAverage);
            baro1Offset = baro1FilterAverage.average;
        }
        baro1Altitude = baro1Raw - baro1Offset;
        
        // Calculate barometer1 offset
        if(!baro2FilterAverage.ready){
            baro2FilterAverage.sample = baroRaw;
            Filter_Average(&baro2FilterAverage);
            baro2Offset = baro2FilterAverage.average;
        }
        baro2Altitude = baroRaw - baro2Offset;
        
        // Calculate range offset
        if(!rangeFilterAverage.ready){
            rangeFilterAverage.sample = rangeRaw;
            Filter_Average(&rangeFilterAverage);
            rangeOffset = rangeFilterAverage.average;
        }
        rangeAltitude = rangeRaw - rangeOffset;
        
        if(accFilterAverage.ready && baro1FilterAverage.ready && baro2FilterAverage.ready){
            
            if(readyy == 0){
                //UART_Print("=========================");
                //UART_Print(" ao: %.4f\n\r", accOffset);
                //UART_Print(" bo: %.4f", baro1Offset);
                //UART_Print(" bo: %.4f", baro2Offset);
                //UART_Print(" ro: %.4f", rangeOffset);
                //UART_Print("\n\r");
                //UART_Print(" baro1Dt: %.8f", baro1Dt);
                //UART_Print(" accDt: %.8f\n\r", accDt);
                readyy = 1;
            }
            // Low pass filter barometric altitude
            baro1AltFilterLowpass.input = 100 * baro1Altitude;
            baro1AltFilterLowpass.dt = baro1Dt;
            Filter_Lowpass(&baro1AltFilterLowpass);
            baro1AltitudeLpf = baro1AltFilterLowpass.output;
            
            baro2AltFilterLowpass.input = 100 * baro2Altitude;
            baro2AltFilterLowpass.dt = baro2Dt;
            Filter_Lowpass(&baro2AltFilterLowpass);
            baro2AltitudeLpf = baro2AltFilterLowpass.output;
            
            // TODO: This method does not take into consideration that we might 
            // wanna land somewhere higher/lower than the starting point, so
            // basically this is a pretty shit method without some sort of 
            // offset reset, but will do now for hover tests
            
            // The vl53l0x doesn't need lowpass-filtering, so let's add upp all
            // altitude measuring sensors before it's fused together with
            // accelerometer data
            baroAltitudeTot = (baro1AltitudeLpf + baro1AltitudeLpf) / 2; // TODO: change one baro1 to working baro2
            // If vl53l0x is within range but barometer out of range
            if((rangeAltitude <= MAX_VL53L0X_ALT) && (baroAltitudeTot < MIN_BARO_ALT)){
                sensorsAltitude = rangeAltitude;
                sensorsDt = rangeDt;
            }
            // If vl53l0x is out of range but barometer within range
            if((baroAltitudeTot >= MIN_BARO_ALT) && (rangeAltitude > MAX_VL53L0X_ALT)){
                sensorsAltitude = rangeAltitude;
                sensorsDt = baro1Dt;
            }
            // If vl53l0x and barometer within range, combine them in a 50cm transition
            if((baroAltitudeTot >= MIN_BARO_ALT) && (rangeAltitude <= MAX_VL53L0X_ALT)){
                // Calculate a damping value between 0 and 1 (0 - 50 / 50)
                float damping = (float)((sensorsAltitude - MIN_BARO_ALT) / (MAX_VL53L0X_ALT - MIN_BARO_ALT));
                // The closer to 0, the bigger the contribution of range, the closer to 1, the bigger the contribution from baro
                sensorsAltitude = Filter_Transition(rangeAltitude, baroAltitudeTot, damping);
                sensorsDt = rangeDt;
            }
                
            // Derive altitude and get velocity in cm/s
            static float baro1AltitudeLast;
            baro1Velocity = (baro1AltitudeLpf - baro1AltitudeLast) / baro1Dt;
            baro1AltitudeLast = baro1AltitudeLpf;
            
            // Derive sensors altitude and get velocity in cm/s
            // TODO: find a better way to determine sensorsDt, since baro is
            // sampled in 50Hz and vl53l0x in 25Hz
            static float sensorsAltitudeLast;
            sensorsVelocity = (sensorsAltitude - sensorsAltitudeLast) / sensorsDt;
            sensorsAltitudeLast = sensorsAltitude;
            
            sensorsVelFilterLowpass.input = sensorsVelocity;
            sensorsVelFilterLowpass.dt = sensorsDt;
            Filter_Lowpass(&sensorsVelFilterLowpass);
            sensorsVelocityLpf = sensorsVelFilterLowpass.output;
            
            // Low pass filter barometric velocity signal
            baro1VelFilterLowpass.input = baro1Velocity;
            baro1VelFilterLowpass.dt = baro1Dt;
            Filter_Lowpass(&baro1VelFilterLowpass);
            baro1VelocityLpf = baro1VelFilterLowpass.output;
            
            // Low pass filter accelerometer acceleration
            accAccFilterLowpass.input = accAcceleration;
            accAccFilterLowpass.dt = accDt;
            Filter_Lowpass(&accAccFilterLowpass);
            accAccelerationLpf = accAccFilterLowpass.output;
            
            //-------------------- Only for show
            static float accVelLast;
            accVel = accVelLast + accAccelerationLpf * accDt;
            accVelLast = accVel;
            
            static float accAltitudeLast;
            accAltitude = accAltitudeLast + accVel * accDt + 0.5f * accAccelerationLpf * accDt * accDt;
            accAltitudeLast = accAltitude;
            //--------------------
            
            // Velocity = initial velocity +  (acceleration * time)
            tempVelocity = velocity + (accAccelerationLpf * accDt); 
            // Position = initial position + (initial velocity * time) + (0.5 * acceleration * (time^2))
            tempAltitude = altitude + (tempVelocity * accDt) + (0.5 * accAccelerationLpf * accDt * accDt);
            // Calculate velocity using complimentary filter 
            static float velCoeff = 0.999;
            velocity = velCoeff * tempVelocity + (1 - velCoeff) * baro1VelocityLpf;
            // Calculate altitude with complimentary filter
            static float altCoeff = 0.995;
            altitude = altCoeff * tempAltitude + (1 - altCoeff) * baro1AltitudeLpf;
            
            // TODO: This block uses the combined laser and barometer data and can
            // probably be used instead of the block above as soon as it's
            // verified :) ---------------------
            // Velocity = initial velocity +  (acceleration * time)
            tempVelocity2 = velocity2 + (accAccelerationLpf * accDt); 
            // Position = initial position + (initial velocity * time) + (0.5 * acceleration * (time^2))
            tempAltitude2 = altitude2 + (tempVelocity2 * accDt) + (0.5 * accAccelerationLpf * accDt * accDt);
            // Calculate velocity using complimentary filter 
            static float vel2Coeff = 0.99;
            velocity2 = vel2Coeff * tempVelocity2 + (1 - vel2Coeff) * sensorsVelocityLpf;
            // Calculate altitude with complimentary filter
            static float alt2Coeff = 0.95;
            altitude2 = alt2Coeff * tempAltitude2 + (1 - alt2Coeff) * sensorsAltitude;
            //------------------------------
        }
        // TODO: assign data with laser
        alt_quad_ptr->velocity = velocity;
        alt_quad_ptr->altitude = altitude;
        alt_quad_ptr->dt = accDt;
        //
        osMailPut(myMailAltToQuadHandle, alt_quad_ptr);
        /*MS5803Event = osMailGet(myMailMS5803ToAltHandle, 0);
        if (MS5803Event.status == osEventMail) {
        ms5803_ptr = MS5803Event.value.p;
        
        baro2 = ms5803_ptr->altitude;
        
        osMailFree(myMailMS5803ToAltHandle, ms5803_ptr);
    }*/
        
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    }
}


static uint32_t counter = 0;
void TelemetryStartTask2(void const * argument)
{
    osDelay(6000);
	while(1){
        osDelay(40); // TODO: osDelayUntil
        if(counter < 250){
            totDt += accDt;
            counter++;
            //UART_Print("Total %d", xPortGetMinimumEverFreeHeapSize());
            //UART_Print(" pitch: %.4f", pitch);
            //UART_Print(" roll: %.4f", roll);
            //UART_Print(" yaw: %.4f", yaw);
            //UART_Print(" gx: %.4f", gyro_x);
            //UART_Print(" gy: %.4f", gyro_y);
            //UART_Print(" gz: %.4f", gyro_z);
            //UART_Print(" b: %.4f", baro1Raw);
            //UART_Print(" b2: %.4f", baro2);
            
            //----- Print to plot
            //UART_Print(" %.4f", totDt);
            //UART_Print(" %.4f", accAltitude);
            //UART_Print(" %.4f", 100 * baro1Altitude);
            //UART_Print(" %.4f", altitude);
            //------
            
            //----- Print to plot
            //UART_Print(" %.4f", totDt);
            //UART_Print(" %.4f", baro1AltitudeLpf);
            //UART_Print(" %.4f", baro2AltitudeLpf);
            //UART_Print(" %.4f", baroAltitudeTot);
            //------
            
            //----- Print to plot
            UART_Print(" %.4f", totDt);
            UART_Print(" %.4f", accAltitude);
            UART_Print(" %.4f", rangeAltitude);
            UART_Print(" %.4f", altitude2);
            //------
            
            //UART_Print(" %.4f", totDt);
            //UART_Print(" %.4f", rangeRaw);
            
            //UART_Print(" ms5803 %.4f", baro2Altitude);
            
            //UART_Print(" bo: %.4f", baro1Offset);
            //UART_Print(" ao: %.4f", acc_z_offset);
            //UART_Print(" dt: %.4f", dt);
            //UART_Print(" alt: %.4f", altitude_data.altitude);
            //UART_Print(" vel: %.4f", altitude_data.velocity);
            UART_Print("\n\r");
        }
    }
}