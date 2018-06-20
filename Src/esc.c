#include "stm32f4xx_hal.h"
#include "hardware.h"
#include <stdint.h>
#include "esc.h"

//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define ESC_THRUST 0
// Units of micro seconds
#define ESC_MIN 1173
#define ESC_MAX 1860
#define ESC_INIT_LOW 0
#define ESC_INIT_HIGH 850

#define TIM_PRESCALER 6

//static uint32_t pwm_min, pwm_max;

void ESC_SetSpeed(uint32_t channel, float speed)
{
    // Calculate min and max pwm value from micro seconds
    uint32_t pwm_min = ((SystemCoreClock/1000000) / (2 * TIM_PRESCALER)) * ESC_MIN;
    uint32_t pwm_max = ((SystemCoreClock/1000000) / (2 * TIM_PRESCALER)) * ESC_MAX;
    // Contrain speed to a value between 0 and 1
    //constrain(speed, 0.0f, 1.0f);
    // Calculate the PWM value from micro seconds
    uint32_t pwm = pwm_min + (uint32_t)(speed * (pwm_max - pwm_min));
    __HAL_TIM_SET_COMPARE(&htim4, channel, pwm);
}


void ESC_Init(uint32_t channel)
{
   
    HAL_TIM_PWM_Start(&htim4, channel);
    HAL_Delay(500); // Maybe not needed
	uint32_t pwm = (((SystemCoreClock/1000000) / (2 * TIM_PRESCALER)) * (ESC_INIT_HIGH));
    __HAL_TIM_SET_COMPARE(&htim4, channel, pwm);
    //uart_printf("pin: %d pwm: %d\n\r", esc->pin_number, ((SystemCoreClock/1000000) / (2 * prescaler)) * (ESC_INIT_HIGH));
	HAL_Delay(2500);
    // Set PWM to 0 before returning from function
    __HAL_TIM_SET_COMPARE(&htim4, channel, 0);
}