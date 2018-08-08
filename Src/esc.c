#include "stm32f4xx_hal.h"
#include "hardware.h"
#include <stdint.h>
#include "esc.h"
#include "functions.h"

// Units of micro seconds
#define ESC_MIN 1000//1173
#define ESC_MAX 2000//1860
#define ESC_INIT_LOW 0
#define ESC_INIT_HIGH 850

#define TIM_PRESCALER 7 //12

void ESC_SetSpeed(uint32_t channel, float speed)
{
    // Calculate min and max pwm value from micro seconds
    uint32_t pwm_min = ((SystemCoreClock/1000000) / (1 * TIM_PRESCALER)) * ESC_MIN;
    uint32_t pwm_max = ((SystemCoreClock/1000000) / (1 * TIM_PRESCALER)) * ESC_MAX;

    // Calculate the PWM value from micro seconds and constrain speed
    uint32_t pwm = pwm_min + (uint32_t)(constrain(speed, 0.0f, 1.0f) * (pwm_max - pwm_min));
    __HAL_TIM_SET_COMPARE(&htim1, channel, pwm);
}


void ESC_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_Delay(500); // Maybe not needed
    
	uint32_t pwm = (((SystemCoreClock/1000000) / (1 * TIM_PRESCALER)) * (ESC_INIT_HIGH));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
	HAL_Delay(2500);
    
    pwm = ((SystemCoreClock/1000000) / (1 * TIM_PRESCALER)) * ESC_MIN;
    // Set PWM to 0 before returning from function
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
    HAL_Delay(2000);
}