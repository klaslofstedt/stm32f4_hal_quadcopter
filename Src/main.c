
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

#include "hardware.h"
#include "em7180.h"
#include "uart_print.h"
#include "attitude.h"
#include "altitude.h"
#include "heading.h"
#include "ms5803.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// Queues
xQueueHandle xQueueEM7180ToQuad;
xQueueHandle xQueueEM7180ToAlt;
xQueueHandle xQueueAltToQuad;
xQueueHandle xQueueMS5803ToAlt;

// Semaphores
xSemaphoreHandle xSemaphoreEM7180Interrupt;

// Mutexes
//SemaphoreHandle_t xMutexI2C;


#ifndef M_PI
#define M_PI 3.14159265358979
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void QuadcopterTask(void *pvParameters);
static void TelemetryTask(void *pvParameters);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM9_Init();
    MX_TIM12_Init();
    MX_TIM13_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    
    // Initialize EM7180 (gyro, acc, mag, baro)
    if(EM7180_Init()){ // TODO: Init with i2c struct, interrupt and Hz
        UART_Print("EM7180 Initialized\n\r");
    }else{ // Print error message
        UART_Print(EM7180_getErrorString());
    }
    // Initialize MS5803 (baro)
    if(MS5803_Init()){ // TODO: Init with i2c
        UART_Print("MS5803 Initialized\n\r");
    }else{ // Print error message
        UART_Print(MS5803_getErrorString());
    }
    // Initialize laser
    
    // Initialize GPS
    
    
    /* USER CODE END 2 */
    
    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    //xMutexI2C = xSemaphoreCreateMutex();
    /* USER CODE END RTOS_MUTEX */
    
    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    vSemaphoreCreateBinary(xSemaphoreEM7180Interrupt);
    //vSemaphoreCreateBinary(xSemaphoreEM7180AltInterrupt);
    /* USER CODE END RTOS_SEMAPHORES */
    
    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */
    
    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
    
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    //xTaskCreate(LedTask, (const char* const)"LedTask", configMINIMAL_STACK_SIZE, 0, osPriorityNormal, 0);
    xTaskCreate(EM7180Task, (const char* const)"EM7180Task", 4*configMINIMAL_STACK_SIZE, 0, 1, 0);
    xTaskCreate(AltitudeTask, (const char* const)"AltitudeTask", 2*configMINIMAL_STACK_SIZE, 0, 1, 0);
    //xTaskCreate(MS5803Task, (const char* const)"MS5803Task", 2*configMINIMAL_STACK_SIZE, 0, 1, 0);
    //xTaskCreate(TelemetryTask, (const char* const)"TelemetryTask", 2*configMINIMAL_STACK_SIZE, 0, 0, 0);
    xTaskCreate(TelemetryTask2, (const char* const)"TelemetryTask2", 2*configMINIMAL_STACK_SIZE, 0, 0, 0);
    xTaskCreate(QuadcopterTask, (const char* const)"QuadcopterTask", 2*configMINIMAL_STACK_SIZE, 0, 1, 0);
    /* USER CODE END RTOS_THREADS */
    
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    xQueueEM7180ToQuad = xQueueCreate(1, sizeof(attitude_data_t));
    xQueueEM7180ToAlt = xQueueCreate(1, sizeof(altitude_data_t));
    xQueueAltToQuad = xQueueCreate(1, sizeof(altitude_hold_data_t));
    xQueueMS5803ToAlt = xQueueCreate(1, sizeof(float));
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
        
    }
    /* USER CODE END 3 */
    
}

/* USER CODE BEGIN 4 */

// green LD4_Pin GPIO_PIN_12 altitude
// orange LD3_Pin GPIO_PIN_13 em7180
// red LD5_Pin GPIO_PIN_14  ms5803
// blue LD6_Pin GPIO_PIN_15 quadcopter

// PA2 - bluetooth rx
// PA3 - bluetooth tx

static attitude_data_t attitude_data;
static altitude_data_t altitude_data;

void QuadcopterTask(void *pvParameters)
{
    
    
    while(1){
        if(xQueueReceive(xQueueEM7180ToQuad, &attitude_data, portMAX_DELAY)){
            HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
            //HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
            // Altitude
            if(xQueueReceive(xQueueAltToQuad, &altitude_data, 0)){
                // Do stuff
            }
            HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);           
        }
    }
}


//float qw, qx, qy, qz;
// Acceleration in m/s^2 adjusted without gravity
//float acc_x, acc_y, acc_z;
// Gyro in m/s (drift free from EM7180?)
//float gyro_x, gyro_y, gyro_z;
// Angles in degrees
//float yaw, pitch, roll;

void TelemetryTask(void *pvParameters)
{
    baro_data_t baro_data;
    
	while(1){
        osDelay(200); // TODO: osDelayUntil
        //UART_Print("Total %d", xPortGetMinimumEverFreeHeapSize());
        //UART_Print(" pitch: %.4f", pitch);
        //UART_Print(" roll: %.4f", roll);
        //UART_Print(" yaw: %.4f", yaw);
        //UART_Print(" gx: %.4f", gyro_x);
        //UART_Print(" gy: %.4f", gyro_y);
        //UART_Print(" gz: %.4f", gyro_z);
        UART_Print(" y: %.4f", attitude_data.quaternions.yaw);
        UART_Print(" p: %.4f", attitude_data.quaternions.pitch);
        UART_Print(" r: %.4f", attitude_data.quaternions.roll);
        //UART_Print(" a1: %.4f", a1);
        //UART_Print(" a2: %.4f", a2);
        //UART_Print(" a3: %.4f", a3);
        UART_Print("\n\r");
    }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
    
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)
    {
        osDelay(100);
    }
    /* USER CODE END 5 */ 
}

/**
* @brief  Period elapsed callback in non blocking mode
* @note   This function is called  when TIM6 interrupt took place, inside
* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
* a global variable "uwTick" used as application time base.
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */
    
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    
    /* USER CODE END Callback 1 */
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
