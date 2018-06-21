#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct{
    uint8_t pin_num;
    uint32_t pwm_read;
    uint32_t pwm_min;
    uint32_t pwm_max;
    TIM_TypeDef *TIMx;
	/*float freq_desired;
    float freq_input;
    float freq_accuracy;
    float duty_max;
    float duty_min;
    float duty_center;
    float duty_thresh;
    float duty_input;
    float scalefactor; // TODO: calculate this instead for set on init
    float output;
    float output_limit;
    float avg[7];*/
} Joystick_t; //Joystick_InitTypeDef

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void Joystick_Init(Joystick_t *joystick, TIM_TypeDef *TIMx);
float Joystick_ReadDuty(Joystick_t *joystick);
uint32_t Joystick_ReadFreq(Joystick_t *joystick);

//float joystick_read_setpoint(joystick_data_t* joystick);
//float joystick_read_thrust(joystick_data_t* joystick);
//float joystick_read_toggle(joystick_data_t* joystick);

#endif