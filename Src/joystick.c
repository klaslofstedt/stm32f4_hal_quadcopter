#include "joystick.h"
// Interrupts
#include "stm32f4xx_it.h"
#include "filter.h"
#include "functions.h"

#define JOYSTICK_READ_MIN       1000
#define JOYSTICK_READ_MAX       2000
#define JOYSTICK_OUTPUT_MIN     -1.0f
#define JOYSTICK_OUTPUT_MAX     1.0f

volatile uint32_t uwIC2Value2 = 0;
volatile uint32_t uwDutyCycle2 = 0;
volatile uint32_t uwFrequency2 = 0;

volatile uint32_t uwIC2Value3 = 0;
volatile uint32_t uwDutyCycle3 = 0;
volatile uint32_t uwFrequency3 = 0;

volatile uint32_t uwIC2Value4 = 0;
volatile uint32_t uwDutyCycle4 = 0;
volatile uint32_t uwFrequency4 = 0;

volatile uint32_t uwIC2Value5 = 0;
volatile uint32_t uwDutyCycle5 = 0;
volatile uint32_t uwFrequency5 = 0;

volatile uint32_t uwIC2Value9 = 0;
volatile uint32_t uwDutyCycle9 = 0;
volatile uint32_t uwFrequency9 = 0;

volatile uint32_t uwIC2Value12 = 0;
volatile uint32_t uwDutyCycle12 = 0;
volatile uint32_t uwFrequency12 = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        // TIM2
        if(htim->Instance == TIM2){
            /* Get the Input Capture value */
            uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            
            if (uwIC2Value2 != 0)
            {
                uwFrequency2 = SystemCoreClock / (2 * 1000* uwIC2Value2);
                uint32_t freqScale = 1000 / uwFrequency2;
                uwDutyCycle2 = freqScale * (1000 - (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) * 1000) / uwIC2Value2);
            }
            else
            {
                uwDutyCycle2 = 0;
                uwFrequency2 = 0;
            }
        }
        // TIM3
        if(htim->Instance == TIM3){
            /* Get the Input Capture value */
            uwIC2Value3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            
            if (uwIC2Value3 != 0)
            {
                uwFrequency3 = SystemCoreClock / (2 * 1000* uwIC2Value3);
                uint32_t freqScale = 1000 / uwFrequency3;
                uwDutyCycle3 = freqScale * (1000 - (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) * 1000) / uwIC2Value3);
            }
            else
            {
                uwDutyCycle3 = 0;
                uwFrequency3 = 0;
            }
        }
        // TIM4
        if(htim->Instance == TIM4){
            /* Get the Input Capture value */
            uwIC2Value4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            
            if (uwIC2Value4 != 0)
            {
                uwFrequency4 = SystemCoreClock / (2 * 1000* uwIC2Value4);
                uint32_t freqScale = 1000 / uwFrequency4;
                uwDutyCycle4 = freqScale * (1000 - (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) * 1000) / uwIC2Value4);
            }
            else
            {
                uwDutyCycle4 = 0;
                uwFrequency4 = 0;
            }
        }
        // TIM5
        if(htim->Instance == TIM5){
            /* Get the Input Capture value */
            uwIC2Value5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            
            if (uwIC2Value5 != 0)
            {
                uwFrequency5 = SystemCoreClock / (2 * 1000* uwIC2Value5);
                uint32_t freqScale = 1000000 / uwFrequency5;
                uwDutyCycle5 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) * freqScale / uwIC2Value5;
            }
            else
            {
                uwDutyCycle5 = 0;
                uwFrequency5 = 0;
            }
        }
        // TIM9
        if(htim->Instance == TIM9){
            /* Get the Input Capture value */
            uwIC2Value9 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            
            if (uwIC2Value9 != 0)
            {
                uwFrequency9 = SystemCoreClock / (1 * 1000* uwIC2Value9);
                uint32_t freqScale = 1000 / uwFrequency9;
                uwDutyCycle9 = freqScale * (1000 - (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) * 1000) / uwIC2Value9);
            }
            else
            {
                uwDutyCycle9 = 0;
                uwFrequency9 = 0;
            }
        }
        // TIM12
        if(htim->Instance == TIM12){
            /* Get the Input Capture value */
            uwIC2Value12 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            
            if (uwIC2Value12 != 0)
            {
                uwFrequency12 = SystemCoreClock / (2 * 1000* uwIC2Value12);
                uint32_t freqScale = 1000 / uwFrequency12;
                uwDutyCycle12 = freqScale * (1000 - (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) * 1000) / uwIC2Value12);
            }
            else
            {
                uwDutyCycle12 = 0;
                uwFrequency12 = 0;
            }
        }
    }
}


uint32_t Joystick_ReadFreq(Joystick_t *joystick)
{
    if(joystick->TIMx == TIM2){
        return uwFrequency2;
    }
    if(joystick->TIMx == TIM3){
        return uwFrequency3;
    }
    if(joystick->TIMx == TIM4){
        return uwFrequency4;
    }
    if(joystick->TIMx == TIM5){
        return uwFrequency5;
    }
    if(joystick->TIMx == TIM9){
        return uwFrequency9;
    }
    if(joystick->TIMx == TIM12){
        return uwFrequency12;
    }
    return 0;
}

float Joystick_ReadDuty(Joystick_t *joystick)
{
    if(joystick->TIMx == TIM2){
        return uwDutyCycle2;
        //return mapf((float)uwDutyCycle2, JOYSTICK_READ_MIN, JOYSTICK_READ_MAX, JOYSTICK_OUTPUT_MIN, JOYSTICK_OUTPUT_MAX);
    }
    if(joystick->TIMx == TIM3){
        return uwDutyCycle3;
        //return mapf((float)uwDutyCycle3, JOYSTICK_READ_MIN, JOYSTICK_READ_MAX, JOYSTICK_OUTPUT_MIN, JOYSTICK_OUTPUT_MAX);
    }
    if(joystick->TIMx == TIM4){
        return uwDutyCycle4;
        //return mapf((float)uwDutyCycle4, JOYSTICK_READ_MIN, JOYSTICK_READ_MAX, JOYSTICK_OUTPUT_MIN, JOYSTICK_OUTPUT_MAX);
    }
    if(joystick->TIMx == TIM5){
        return uwDutyCycle5;
        //return mapf((float)uwDutyCycle5, JOYSTICK_READ_MIN, JOYSTICK_READ_MAX, JOYSTICK_OUTPUT_MIN, JOYSTICK_OUTPUT_MAX);
    }
    if(joystick->TIMx == TIM9){
        return uwDutyCycle9;
        //return mapf((float)uwDutyCycle9, JOYSTICK_READ_MIN, JOYSTICK_READ_MAX, JOYSTICK_OUTPUT_MIN, JOYSTICK_OUTPUT_MAX);
    }
    if(joystick->TIMx == TIM12){
        return uwDutyCycle12;
        //return mapf((float)uwDutyCycle12, JOYSTICK_READ_MIN, JOYSTICK_READ_MAX, JOYSTICK_OUTPUT_MIN, JOYSTICK_OUTPUT_MAX);
    }
    return 0;
}

void Joystick_Init(Joystick_t *joystick, TIM_TypeDef *TIMx)
{
    joystick->TIMx = TIMx;
}


/*static int8_t joystick_accuracy(float val, float ref, float step);
static void joystick_filter(joystick_data_t* in);


static void joystick_filter(joystick_data_t* in)
{
in->avg[0] = in->duty_input;
// Calculate a moving average. Every term must add up to (2520*in->avg[n] / 2520), hence the odd last term.
in->avg[0] = in->avg[0]/4 + in->avg[1]/5 + in->avg[2]/6 + in->avg[3]/7 + in->avg[4]/8 + in->avg[5]/9 + (11*in->avg[6]/2520);
// Move the average
in->avg[6] = in->avg[5];
in->avg[5] = in->avg[4];
in->avg[4] = in->avg[3];
in->avg[3] = in->avg[2];
in->avg[2] = in->avg[1];
in->avg[1] = in->avg[0];

in->duty_input = in->avg[0];
}

static int8_t joystick_accuracy(float val, float ref, float step)
{
if(val < (ref + step) && val > (ref - step)){
return 0;
    }
    else if(val <= (ref - step)){
return -1;
    }
    else{
// when val >= (ref + 1)
return 1;
    }
}


float joystick_read_setpoint(joystick_data_t* in)
{
in->freq_input = isr_read_freq(in->pin_num);
in->duty_input = isr_read_duty(in->pin_num);
joystick_filter(in);

float output = 0;
if(joystick_accuracy(in->freq_input, in->freq_desired, in->freq_accuracy) == 0) {
if(joystick_accuracy(in->duty_input, in->duty_center, in->duty_thresh) == -1){
output = (in->scalefactor * (in->duty_center - in->duty_input));
        }
        else if(joystick_accuracy(in->duty_input, in->duty_center, in->duty_thresh) == 1){
output = -(in->scalefactor * (in->duty_input - in->duty_center));
        }
        else{
output = 0;
        }
    }
if(output > in->output_limit){
output = in->output_limit;
    }
if(output < -in->output_limit){
output = -in->output_limit;
    }
return output;
}

float joystick_read_thrust(joystick_data_t* in)
{
in->freq_input = isr_read_freq(in->pin_num);
in->duty_input = isr_read_duty(in->pin_num);
joystick_filter(in);

float output = 0;
if(joystick_accuracy(in->freq_input, in->freq_desired, in->freq_accuracy) == 0) {
if(joystick_accuracy(in->duty_input, in->duty_max, in->duty_thresh) == -1){
output = -(in->scalefactor * (in->duty_input - in->duty_max));
        }
        else{
output = 0;
        }
    }
if(output > in->output_limit){
output = in->output_limit;
    }
return output;
}

float joystick_read_toggle(joystick_data_t* in)
{
in->freq_input = isr_read_freq(in->pin_num);
in->duty_input = isr_read_duty(in->pin_num);

float output = 0;
if(joystick_accuracy(in->freq_input, in->freq_desired, in->freq_accuracy) == 0) {
if(in->duty_input > in->duty_center){
output = 1;
        }
        else{
output = 0;
        }
    }
return output;
}*/