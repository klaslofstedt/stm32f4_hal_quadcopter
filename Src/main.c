
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether 
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
*
* Copyright (c) 2018 STMicroelectronics International N.V. 
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted, provided that the following conditions are met:
*
* 1. Redistribution of source code must retain the above copyright notice, 
*    this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 3. Neither the name of STMicroelectronics nor the names of other 
*    contributors to this software may be used to endorse or promote products 
*    derived from this software without specific written permission.
* 4. This software, including modifications and/or derivative works of this 
*    software, must execute solely and exclusively on microcontroller or
*    microprocessor devices manufactured by or for STMicroelectronics.
* 5. Redistribution and use of this software other than as permitted under 
*    this license is void and will automatically terminate your rights under 
*    this license. 
*
* THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
* PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
* RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
* SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "math.h"

#include "stm32f4xx_it.h"

#include "hardware.h"
#include "em7180.h"
#include "uart_print.h"
#include "attitude.h"
#include "altitude.h"
#include "heading.h"
#include "ms5803.h"
#include "pid.h"
#include "esc.h"

#include "vl53l0x.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "types.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

osThreadId EM7180TaskHandle;
osThreadId AltitudeTaskHandle;
osThreadId QuadcopterTaskHandle;
osThreadId TelemetryTaskHandle;
osThreadId MS5803TaskHandle;
osThreadId TelemetryTask2Handle;
osThreadId VL53L0XTaskHandle;
osSemaphoreId myBinarySemEM7180InterruptHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osMailQDef(myMailEM7180ToQuadHandle, 1, em7180_attitude_data_t);
osMailQId myMailEM7180ToQuadHandle;

osMailQDef(myMailEM7180ToAltHandle, 1, em7180_altitude_data_t);
osMailQId myMailEM7180ToAltHandle;

osMailQDef(myMailMS5803ToAltHandle, 1, ms5803_altitude_data_t);
osMailQId myMailMS5803ToAltHandle;

osMailQDef(myMailAltToQuadHandle, 1, altitude_data_t);
osMailQId myMailAltToQuadHandle;

osMailQDef(myMailVL53L0XToAltHandle, 1, vl53l0x_range_data_t);
osMailQId myMailVL53L0XToAltHandle;


#ifndef M_PI
#define M_PI 3.14159265358979
#endif


//#define TIMx                           TIM2
//#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()

/* Definition for TIMx Pins */
//#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOA_CLK_ENABLE()
//#define GPIO_PORT                      GPIOA
//#define GPIO_PIN_CHANNEL2              GPIO_PIN_1
//#define GPIO_AF_TIMx                   GPIO_AF2_TIM2

/* Definition for TIMx's NVIC */
//#define TIMx_IRQn                      TIM2_IRQn
//#define TIMx_IRQHandler                TIM2_IRQHandler

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void EM7180StartTask(void const * argument);
void AltitudeStartTask(void const * argument);
void QuadcopterStartTask(void const * argument);
void TelemetryStartTask(void const * argument);
void MS5803StartTask(void const * argument);
void TelemetryStartTask2(void const * argument);
void VL53L0XStartTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

TIM_HandleTypeDef        TimHandle;

/* Timer Input Capture Configuration Structure declaration */
TIM_IC_InitTypeDef       sConfig;

/* Slave configuration structure */
TIM_SlaveConfigTypeDef   sSlaveConfig;
/* Captured Value */
__IO uint32_t            uwIC2Value = 0;
/* Duty Cycle Value */
__IO uint32_t            uwDutyCycle = 0;
/* Frequency Value */
__IO uint32_t            uwFrequency = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        /* Get the Input Capture value */
        uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        
        if (uwIC2Value != 0)
        {
            /* uwFrequency computation
            TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */
            uwFrequency = SystemCoreClock / (2 * 1000* uwIC2Value);
            //uwFrequency = (HAL_RCC_GetHCLKFreq())/2 / uwIC2Value;
            
            /* Duty cycle computation */
            uwDutyCycle = ((1000 - (((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 1000) / (uwIC2Value))) * 1000 / uwFrequency);
            //uwDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 1000*1000) / (uwIC2Value * uwFrequency);
            
            
        }
        else
        {
            uwDutyCycle = 0;
            uwFrequency = 0;
        }
    }
}
/* USER CODE END 0 */

/**
* @brief  The application entry point.
*
* @retval None
*/
int main(void)
{
    /* USER CODE BEGIN 1 */
    
    /* USER CODE END 1 */
    
    /* MCU Configuration----------------------------------------------------------*/
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    
    /* USER CODE BEGIN Init */
    
    /* USER CODE END Init */
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* USER CODE BEGIN SysInit */
    
    /* USER CODE END SysInit */
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C3_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    MX_UART5_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    
    
    // Initialize EM7180 (gyro, acc, mag, baro)
    if(EM7180_Init()){ // TODO: Init with i2c struct, interrupt and Hz
        UART_Print("EM7180 Initialized\n\r");
    }else{ // Print error message
        UART_Print(EM7180_getErrorString());
    }
    
    // Initialize VL53L0X (laser)
    if(VL53L0X_Init()){ // TODO: Init with i2c struct, interrupt and Hz
        UART_Print("VL53L0X Initialized\n\r");
    }else{ // Print error message
        UART_Print("VL53L0X Error (but will likely work anyway)\n\r");
        //UART_Print(EM7180_getErrorString());
    }
    
    // Initialize MS5803 (baro)
    if(MS5803_Init()){ // TODO: Init with i2c
        UART_Print("MS5803 Initialized\n\r");
    }else{ // Print error message
        UART_Print(MS5803_getErrorString());
    }
    
    //HAL_TIM_Base_Start_IT(&htim1);
    //HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    //HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    // Initialize ESCs
    // Prescaler = 6-1;
    // Period = (((SystemCoreClock / 2) / 400) / 6) -1;
    ESC_Init(TIM_CHANNEL_1);
    ESC_Init(TIM_CHANNEL_2);
    //ESC_Init(TIM_CHANNEL_3);
    //ESC_Init(TIM_CHANNEL_4);
    
    //TimHandle.Instance = TIMx;
    
    /* Initialize TIMx peripheral as follow:
    + Period = 0xFFFF
    + Prescaler = 0
    + ClockDivision = 0
    + Counter direction = Up
    */
    /*TimHandle.Init.Period = 0xFFFF;
    TimHandle.Init.Prescaler = 1000;
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;  
    if(HAL_TIM_IC_Init(&TimHandle) != HAL_OK)
    {

        Error_Handler();
    }*/
    
    /*##-2- Configure the Input Capture channels ###############################*/ 
    /* Common configuration */
    //sConfig.ICPrescaler = TIM_ICPSC_DIV1;
    //sConfig.ICFilter = 0;  
    
    /* Configure the Input Capture of channel 1 */
    //sConfig.ICPolarity = TIM_ICPOLARITY_FALLING;
    //sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;    
    //if(HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
    //{
        /* Configuration Error */
        //Error_Handler();
    //}
    
    /* Configure the Input Capture of channel 2 */
    /*sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
    sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    if(HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }*/
    /*##-3- Configure the slave mode ###########################################*/
    /* Select the slave Mode: Reset Mode */
    /*sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger  = TIM_TS_TI2FP2;
    if(HAL_TIM_SlaveConfigSynchronization(&TimHandle, &sSlaveConfig) != HAL_OK)
    {
        
        Error_Handler();
    }*/
    
    /*##-4- Start the Input Capture in interrupt mode ##########################*/
    if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
    
    /*##-5- Start the Input Capture in interrupt mode ##########################*/
    if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
    
    /* USER CODE END 2 */
    
    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */
    
    /* Create the semaphores(s) */
    /* definition and creation of myBinarySemEM7180Interrupt */
    osSemaphoreDef(myBinarySemEM7180Interrupt);
    myBinarySemEM7180InterruptHandle = osSemaphoreCreate(osSemaphore(myBinarySemEM7180Interrupt), 1);
    
    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */
    
    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */
    
    /* Create the thread(s) */
    /* definition and creation of EM7180Task */
    osThreadDef(EM7180Task, EM7180StartTask, osPriorityAboveNormal, 0, 512);
    EM7180TaskHandle = osThreadCreate(osThread(EM7180Task), NULL);
    
    /* definition and creation of AltitudeTask */
    osThreadDef(AltitudeTask, AltitudeStartTask, osPriorityAboveNormal, 0, 512);
    AltitudeTaskHandle = osThreadCreate(osThread(AltitudeTask), NULL);
    
    /* definition and creation of QuadcopterTask */
    osThreadDef(QuadcopterTask, QuadcopterStartTask, osPriorityAboveNormal, 0, 256);
    QuadcopterTaskHandle = osThreadCreate(osThread(QuadcopterTask), NULL);
    
    /* definition and creation of TelemetryTask */
    osThreadDef(TelemetryTask, TelemetryStartTask, osPriorityBelowNormal, 0, 256);
    TelemetryTaskHandle = osThreadCreate(osThread(TelemetryTask), NULL);
    
    /* definition and creation of MS5803Task */
    osThreadDef(MS5803Task, MS5803StartTask, osPriorityNormal, 0, 256);
    MS5803TaskHandle = osThreadCreate(osThread(MS5803Task), NULL);
    
    /* definition and creation of TelemetryTask2 */
    osThreadDef(TelemetryTask2, TelemetryStartTask2, osPriorityBelowNormal, 0, 256);
    //TelemetryTask2Handle = osThreadCreate(osThread(TelemetryTask2), NULL);
    
    /* definition and creation of VL53L0XTask */
    osThreadDef(VL53L0XTask, VL53L0XStartTask, osPriorityNormal, 0, 256);
    VL53L0XTaskHandle = osThreadCreate(osThread(VL53L0XTask), NULL);
    
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
    
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    
    myMailEM7180ToQuadHandle = osMailCreate(osMailQ(myMailEM7180ToQuadHandle), NULL);
    myMailEM7180ToAltHandle = osMailCreate(osMailQ(myMailEM7180ToAltHandle), NULL);
    myMailAltToQuadHandle = osMailCreate(osMailQ(myMailAltToQuadHandle), NULL);
    myMailMS5803ToAltHandle = osMailCreate(osMailQ(myMailMS5803ToAltHandle), NULL);
    myMailVL53L0XToAltHandle = osMailCreate(osMailQ(myMailVL53L0XToAltHandle), NULL);
    /* USER CODE END RTOS_QUEUES */
    
    
    /* Start scheduler */
    osKernelStart();
    
    /* We should never get here as control is now taken by the scheduler */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        
        /* USER CODE END WHILE */
        
        /* USER CODE BEGIN 3 */
        UART_Print("RTOS Error");
        HAL_Delay(1000);
    }
    /* USER CODE END 3 */
    
}



/* USER CODE BEGIN 4 */

// GPIO_PIN_12 altitude
// GPIO_PIN_13 em7180
// GPIO_PIN_14  ms5803
// GPIO_PIN_15 quadcopter

// PA2 - bluetooth rx
// PA3 - bluetooth tx

//float qw, qx, qy, qz;
// Acceleration in m/s^2 adjusted without gravity
//float acc_x, acc_y, acc_z;
// Gyro in m/s (drift free from EM7180?)
//float gyro_x, gyro_y, gyro_z;
// Angles in degrees

static float yaw, pitch, roll;
static float altitude, velocity, dt;
//static float joystick_lx, joystick_ly, joystick_rx, joystick_ry;

/* USER CODE END 4 */


/* QuadcopterStartTask function */
void QuadcopterStartTask(void const * argument)
{
    /* USER CODE BEGIN QuadcopterStartTask */
    ESC_SetSpeed(TIM_CHANNEL_1, 1.0f);
    ESC_SetSpeed(TIM_CHANNEL_2, 0.0f);
    
    em7180_attitude_data_t *attitude_ptr;
    osEvent EM7180Event;
    
    altitude_data_t *altitude_ptr;
    osEvent AltitudeEvent;
    /* Infinite loop */
    while(1){
        // Wait on attitude data from em7180 task
        EM7180Event = osMailGet(myMailEM7180ToQuadHandle, osWaitForever);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        if (EM7180Event.status == osEventMail) {
            attitude_ptr = EM7180Event.value.p;
            
            yaw = attitude_ptr->angle.yaw;
            pitch = attitude_ptr->angle.pitch;
            roll = attitude_ptr->angle.roll;
            
            osMailFree(myMailEM7180ToQuadHandle, attitude_ptr);
        }
        
        // Wait on altitude data from altitude task
        AltitudeEvent = osMailGet(myMailAltToQuadHandle, 0); //osWaitForever
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        if (AltitudeEvent.status == osEventMail) {
            altitude_ptr = AltitudeEvent.value.p;
            
            altitude = altitude_ptr->altitude;
            velocity = altitude_ptr->velocity;
            //dt = altitude_ptr->angle.roll;
            
            osMailFree(myMailAltToQuadHandle, altitude_ptr);
        }
        
        // Read joystick
        //joy_count = __HAL_TIM_GET_COUNTER(&htim1); //TIM_CHANNEL_2);
        //joy_capture = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    /* USER CODE END QuadcopterStartTask */
}

/* TelemetryStartTask function */
void TelemetryStartTask(void const * argument)
{
    /* USER CODE BEGIN TelemetryStartTask */
    /* Infinite loop */
    
    while(1){
        osDelay(200); // TODO: osDelayUntil
        
        //UART_Print(" range %d", range_mm);
        //UART_Print("Total %d", xPortGetMinimumEverFreeHeapSize());
        //UART_Print(" gx: %.4f", gyro_x);
        //UART_Print(" gy: %.4f", gyro_y);
        //UART_Print(" gz: %.4f", gyro_z);
        
        //UART_Print(" y: %.4f", yaw);
        //UART_Print(" p: %.4f", pitch);
        //UART_Print(" r: %.4f", roll);
        //UART_Print(" a: %.4f", altitude);
        
        //input_pwm_ch1 = TIM1->CCR1;
        //input_pwm_ch2 = TIM1->CCR2;
        //input_pwm = (float)(100 * (input_pwm_ch2 / input_pwm_ch1));
        //UART_Print(" a1: %.4f", a1);
        //UART_Print(" a2: %.4f", a2);
        //UART_Print(" a3: %.4f", a3);
        
        // Read joystick
        //joy_count = __HAL_TIM_GET_COUNTER(&htim1);
        
        UART_Print(" pwm: %d", uwDutyCycle);
        UART_Print(" freq: %d", uwFrequency);
        UART_Print("\n\r");
    }
    /* USER CODE END TelemetryStartTask */
}


/**
* @brief  This function is executed in case of error occurrence.
* @param  file: The file name as string.
* @param  line: The line in file as a number.
* @retval None
*/
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
