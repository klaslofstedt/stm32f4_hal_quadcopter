
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
#include "bmp280.h"
#include "bmp180.h"
#include "pid.h"
#include "esc.h"
#include "joystick.h"
#include "filter.h"

#include "vl53l0x.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/*I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;*/

osThreadId EM7180TaskHandle;
osThreadId AltitudeTaskHandle;
osThreadId QuadcopterTaskHandle;
osThreadId TelemetryTaskHandle;
osThreadId MS5803TaskHandle;
osThreadId TelemetryTask2Handle;
osThreadId VL53L0XTaskHandle;
osThreadId BMP180TaskHandle;
osThreadId BMP280TaskHandle;
osMutexId mutexI2C1Handle;
osSemaphoreId binarySemEM7180InterruptHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osMailQDef(mailEM7180ToQuadHandle, 1, Em7180Attitude_t);
osMailQId mailEM7180ToQuadHandle;

osMailQDef(mailEM7180ToAltHandle, 1, Em7180Altitude_t);
osMailQId mailEM7180ToAltHandle;

osMailQDef(mailMS5803ToAltHandle, 1, Ms5803Altitude_t);
osMailQId mailMS5803ToAltHandle;

osMailQDef(mailBMP280ToAltHandle, 1, Bmp280Altitude_t);
osMailQId mailBMP280ToAltHandle;

osMailQDef(mailBMP180ToAltHandle, 1, Bmp180Altitude_t);
osMailQId mailBMP180ToAltHandle;

osMailQDef(mailAltToQuadHandle, 1, Altitude_t);
osMailQId mailAltToQuadHandle;

osMailQDef(mailVL53L0XToAltHandle, 1, Vl53l0xRange_t);
osMailQId mailVL53L0XToAltHandle;

// Joystick objects
Joystick_t yawJoystick;
Joystick_t pitchJoystick;
Joystick_t rollJoystick;
Joystick_t thrustJoystick;
Joystick_t switchlJoystick;
Joystick_t switchrJoystick;

// PID objects
PID_t yawPid = {
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .k_p = -0.0035,
    .k_i = 0.0f,
    .k_d = 0.0
};

PID_t pitchPid = {
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .k_p = 0.0065f,
    .k_i = 0.000001f,
    .k_d = 0.00025f
};

PID_t rollPid = {
    .boundary_max = 1.0f,
    .boundary_min = -1.0f,
    .k_p = 0.0065f,
    .k_i = 0.000001f,
    .k_d = 0.00025f
};

PID_t altitudePid = {
    .boundary_max = 0.3f,
    .boundary_min = -0.3f,
    .k_p = 0.009f,
    .k_i = 0.0f,
    .k_d = 0.8f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void Quadcopter_StartTask(void const * argument);
void Telemetry_StartTask(void const * argument);



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
    MX_TIM4_Init();
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    MX_UART5_Init();
    MX_TIM2_Init();
    MX_TIM1_Init();
    MX_TIM9_Init();
    MX_TIM12_Init();
    MX_TIM3_Init();
    MX_TIM5_Init();
    MX_TIM10_Init();
    /* USER CODE BEGIN 2 */
    
    // PWM Input Timer 2
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    // PWM Input Timer 3
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    // PWM Input Timer 4
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
    // PWM Input Timer 5
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    // PWM Input Timer 9
    HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
    // PWM Input Timer 12
    HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
    
    UART_Print("Peripherals Initiated\n\r");
    // Initialize ESCs TIM1 PE9, PE11, PE13, PE14
    ESC_Init(); // TODO: init with timer?
    //ESC_Init(TIM_CHANNEL_1);
    //ESC_Init(TIM_CHANNEL_2);
    //ESC_Init(TIM_CHANNEL_3);
    //ESC_Init(TIM_CHANNEL_4);
    
    /*ESC_SetSpeed(TIM_CHANNEL_1, 0);
    ESC_SetSpeed(TIM_CHANNEL_2, 0);
    ESC_SetSpeed(TIM_CHANNEL_3, 0);
    ESC_SetSpeed(TIM_CHANNEL_4, 0);*/
    HAL_Delay(2000);
    
    // Initialize joystick
    Joystick_Init(&yawJoystick,     TIM2);  // PA1
    Joystick_Init(&pitchJoystick,   TIM3);  // PA7
    Joystick_Init(&rollJoystick,    TIM4);  // PD13
    Joystick_Init(&thrustJoystick,  TIM5);  // PA0
    Joystick_Init(&switchlJoystick, TIM9);  // PE6
    Joystick_Init(&switchrJoystick, TIM12); // PB15
    
    // Initialize EM7180 (gyro, acc, mag, baro)
    if(EM7180_Init()){ // TODO: Init with i2c3 struct, interrupt and Hz
        UART_Print("EM7180 Initialized\n\r");
    }else{ // Print error message
        UART_Print(EM7180_GetErrorString());
    }
    
    // Initialize VL53L0X (laser)
    if(VL53L0X_Init()){ // TODO: Init with i2c1 struct, interrupt and Hz
        UART_Print("VL53L0X Initialized\n\r");
    }else{ // Print error message
        UART_Print("VL53L0X Error (but will likely work anyway)\n\r");
        //UART_Print(EM7180_GetErrorString());
    }
    
    // Initialize MS5803 (baro)
    /*if(MS5803_Init()){ // TODO: Init with i2c
    UART_Print("MS5803 Initialized\n\r");
}else{ // Print error message
    UART_Print(MS5803_GetErrorString());
}*/
    
    // Initialize BMP280 (baro)
    if(BMP280_Init()){ // TODO: Init with i2c
        UART_Print("BMP280 Initialized\n\r");
    }else{ // Print error message
        UART_Print("BMP280 Error\n\r");
    }
    
    // Initialize BMP180 (baro)
    if(BMP180_Init()){ // TODO: Init with i2c
        UART_Print("BMP180 Initialized\n\r");
    }else{ // Print error message
        UART_Print("BMP180 Error\n\r");
    }
    
    /* USER CODE END 2 */
    
    /* Create the mutex(es) */
    /* definition and creation of mutexI2C1 */
    osMutexDef(mutexI2C1);
    mutexI2C1Handle = osMutexCreate(osMutex(mutexI2C1));
    
    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */
    
    /* Create the semaphores(s) */
    /* definition and creation of binarySemEM7180Interrupt */
    osSemaphoreDef(binarySemEM7180Interrupt);
    binarySemEM7180InterruptHandle = osSemaphoreCreate(osSemaphore(binarySemEM7180Interrupt), 1);
    
    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */
    
    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */
    
    /* Create the thread(s) */
    /* definition and creation of EM7180Task */
    osThreadDef(EM7180Task, EM7180_StartTask, osPriorityHigh, 0, 512);
    EM7180TaskHandle = osThreadCreate(osThread(EM7180Task), NULL);
    
    /* definition and creation of AltitudeTask */
    osThreadDef(AltitudeTask, Altitude_StartTask, osPriorityHigh, 0, 512);
    AltitudeTaskHandle = osThreadCreate(osThread(AltitudeTask), NULL);
    
    /* definition and creation of QuadcopterTask */
    osThreadDef(QuadcopterTask, Quadcopter_StartTask, osPriorityHigh, 0, 256);
    QuadcopterTaskHandle = osThreadCreate(osThread(QuadcopterTask), NULL);
    
    /* definition and creation of TelemetryTask */
    osThreadDef(TelemetryTask, Telemetry_StartTask, osPriorityLow, 0, 256);
    TelemetryTaskHandle = osThreadCreate(osThread(TelemetryTask), NULL);
    
    /* definition and creation of MS5803Task */
    //osThreadDef(MS5803Task, MS5803_StartTask, osPriorityNormal, 0, 256);
    //MS5803TaskHandle = osThreadCreate(osThread(MS5803Task), NULL);
    
    /* definition and creation of TelemetryTask2 */
    osThreadDef(TelemetryTask2, Telemetry_StartTask2, osPriorityLow, 0, 256);
    //TelemetryTask2Handle = osThreadCreate(osThread(TelemetryTask2), NULL);
    
    /* definition and creation of VL53L0XTask */
    osThreadDef(VL53L0XTask, VL53L0X_StartTask, osPriorityAboveNormal, 0, 256);
    VL53L0XTaskHandle = osThreadCreate(osThread(VL53L0XTask), NULL);
    
    /* definition and creation of BMP180Task */
    osThreadDef(BMP180Task, BMP180_StartTask, osPriorityBelowNormal, 0, 256);
    BMP180TaskHandle = osThreadCreate(osThread(BMP180Task), NULL);
    
    /* definition and creation of BMP280Task */
    osThreadDef(BMP280Task, BMP280_StartTask, osPriorityNormal, 0, 256);
    BMP280TaskHandle = osThreadCreate(osThread(BMP280Task), NULL);
    
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
    
    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    
    mailEM7180ToQuadHandle    = osMailCreate(osMailQ(mailEM7180ToQuadHandle), NULL);
    mailEM7180ToAltHandle     = osMailCreate(osMailQ(mailEM7180ToAltHandle), NULL);
    mailAltToQuadHandle       = osMailCreate(osMailQ(mailAltToQuadHandle), NULL);
    mailMS5803ToAltHandle     = osMailCreate(osMailQ(mailMS5803ToAltHandle), NULL);
    mailBMP280ToAltHandle     = osMailCreate(osMailQ(mailBMP280ToAltHandle), NULL);
    mailBMP180ToAltHandle     = osMailCreate(osMailQ(mailBMP180ToAltHandle), NULL);
    mailVL53L0XToAltHandle    = osMailCreate(osMailQ(mailVL53L0XToAltHandle), NULL);
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

float yawAngle, pitchAngle, rollAngle;
float yawRate, pitchRate, rollRate;
float altitude, altitudeVelocity;
float attitudeDt, altitudeDt;
float hover = 0.55f;
bool arm = false;

float esc1, esc2, esc3, esc4;

/* Quadcopter_StartTask function */
void Quadcopter_StartTask(void const * argument)
{
    /* USER CODE BEGIN Quadcopter_StartTask */
    
    // Allocate space for queued attitude struct
    static Em7180Attitude_t *pEm7180Attitude;
    osEvent EM7180Event;
    // Allocate space for queued altitude struct
    static Altitude_t *pAltitude;
    osEvent AltitudeEvent;
    
    /* Infinite loop */
    while(1){
        // Wait on attitude data from em7180 task (250 Hz)
        EM7180Event = osMailGet(mailEM7180ToQuadHandle, osWaitForever);
        // Turn on LED
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        // Evaluate event status
        if (EM7180Event.status == osEventMail) {
            pEm7180Attitude = EM7180Event.value.p;
            // Get angle (degrees)
            yawAngle    = pEm7180Attitude->angle.yaw;
            pitchAngle  = pEm7180Attitude->angle.pitch;
            rollAngle   = pEm7180Attitude->angle.roll;
            // Get angular rate (degrees/s) 
            yawRate     = pEm7180Attitude->gyro.z;
            pitchRate   = pEm7180Attitude->gyro.y;
            rollRate    = pEm7180Attitude->gyro.x;
            // Get dt timestamp (seconds)
            attitudeDt  = pEm7180Attitude->dt;
            // Free the allocated struct memory
            osMailFree(mailEM7180ToQuadHandle, pEm7180Attitude);
        }
        
        // Poll altitude data from altitude task (50 Hz)
        AltitudeEvent = osMailGet(mailAltToQuadHandle, 0);
        if (AltitudeEvent.status == osEventMail) {
            pAltitude = AltitudeEvent.value.p;
            // Get the altitude (cm)
            altitude = pAltitude->altitude;
            // Get the altitudal velocity (cm/s?)
            altitudeVelocity = pAltitude->velocity;
            // Get dt timestamp (seconds)
            altitudeDt  = (uint32_t)pAltitude->dt;
            // Free the allocated struct memory
            osMailFree(mailAltToQuadHandle, pAltitude);
        }
        
        // Build yaw PID object
        // setpoint for yaw must be a counter ++ -- rather than a value
        //yawPid.setpoint   = 0;//Joystick_ReadDuty(&yawJoystick);
        if(yawAngle > 180){
            yawAngle = yawAngle - 360;
            yawRate = -yawRate;
        }
        yawPid.input      = yawAngle;
        yawPid.rate       = yawRate;
        yawPid.dt         = attitudeDt;
        // Build pitch PID object
        pitchPid.setpoint = 0;//Joystick_ReadDuty(&pitchJoystick);
        pitchPid.input    = pitchAngle;
        pitchPid.rate     = pitchRate;
        pitchPid.dt       = attitudeDt;
        // Build roll PID object
        rollPid.setpoint  = 0;//Joystick_ReadDuty(&rollJoystick);
        rollPid.input     = rollAngle;
        rollPid.rate      = rollRate;
        rollPid.dt        = attitudeDt;
        // TODO: joystick thrust cannot return -1 to 1 but rather a higher/lower
        // setpoint value
        altitudePid.setpoint = 50;//Joystick_ReadDuty(&thrustJoystick);
        altitudePid.input    = altitude;
        altitudePid.rate     = altitudeVelocity;
        altitudePid.dt       = altitudeDt;
        
        // Calculate PID values
        float pitch     = PID_Calc(&pitchPid);
        float roll      = PID_Calc(&rollPid);
        float yaw       = PID_Calc(&yawPid);
        float alt       = 0;//PID_Calc(&altitudePid);
        float x         = 0; //xPid.output;
        float y         = 0; //yPid.output;

        
        // Calculate the motor values
        // 1  front  4
        // left  right
        // 2  back   3
        esc1 = hover + alt + roll + pitch + yaw + x + y;
        esc2 = hover + alt + roll - pitch - yaw - x + y;
        esc3 = hover + alt - roll - pitch + yaw - x - y;
        esc4 = hover + alt - roll + pitch - yaw + x - y;
        
        // Feed motors only if armed
        if(arm){
            //
            ESC_SetSpeed(TIM_CHANNEL_1, esc1);
            ESC_SetSpeed(TIM_CHANNEL_2, esc2);
            ESC_SetSpeed(TIM_CHANNEL_3, esc3);
            ESC_SetSpeed(TIM_CHANNEL_4, esc4);
        }else{
            // In order for the drone not to spin around yaw axis on ARM, set 
            // the first setpoint value to the actual reading
            yawPid.setpoint = yawAngle;
            // No motor speed
            ESC_SetSpeed(TIM_CHANNEL_1, 0);
            ESC_SetSpeed(TIM_CHANNEL_2, 0);
            ESC_SetSpeed(TIM_CHANNEL_3, 0);
            ESC_SetSpeed(TIM_CHANNEL_4, 0);
        }
        
        // Set PID constants
        if(Joystick_ReadDuty(&yawJoystick) < 1200){
            //pitchPid.k_d = pitchPid.k_d - 0.000001;
            //rollPid.k_d = rollPid.k_d - 0.000001;
            yawPid.k_p = yawPid.k_p - 0.000001;
        }
        if(Joystick_ReadDuty(&yawJoystick) > 1700){
            //pitchPid.k_d = pitchPid.k_d + 0.000001;
            //rollPid.k_d = rollPid.k_d + 0.000001;
            yawPid.k_p = yawPid.k_p + 0.000001;
        }
        if(Joystick_ReadDuty(&pitchJoystick) < 1200){
            hover = hover - 0.001;
            //pitchPid.k_p = pitchPid.k_p - 0.00001;
            //rollPid.k_p = rollPid.k_p - 0.00001;
        }
        if(Joystick_ReadDuty(&pitchJoystick) > 1700){
            hover = hover + 0.001;
            //pitchPid.k_p = pitchPid.k_p + 0.00001;
            //rollPid.k_p = rollPid.k_p + 0.00001;
        }
        if(Joystick_ReadDuty(&rollJoystick) < 1200){
            //
            arm = !arm;
            //pitchPid.k_d = pitchPid.k_d - 0.000001;
            //rollPid.k_d = rollPid.k_d - 0.00001;
        }        
        // Turn off LED
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    /* USER CODE END Quadcopter_StartTask */
}

/* Telemetry_StartTask function */
void Telemetry_StartTask(void const * argument)
{
    /* USER CODE BEGIN Telemetry_StartTask */
    /* Infinite loop */
    float totDt = 0;
    uint32_t counter = 0;
    while(1){
        osDelay(500); // TODO: osDelayUntil
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        if(counter < 200000){
            //totDt += pitchPid.dt;
            //counter++;
            //UART_Print(" range %d", range_mm);
            //UART_Print("Total %d", xPortGetMinimumEverFreeHeapSize());
            //UART_Print(" gx: %.4f", gyro_x);
            //UART_Print(" gy: %.4f", gyro_y);
            //UART_Print(" gz: %.4f", gyro_z);
            
            UART_Print(" y: %.4f", yawAngle);
            UART_Print(" ys: %.4f", yawPid.setpoint); 
            //UART_Print(" p: %.4f", pitchAngle);
            //UART_Print(" r: %.4f", rollAngle);
            UART_Print(" a: %.4f", altitude);
            
            //UART_Print(" %.4f", totDt);
            //UART_Print(" %.4f", pitchAngle);
            //UART_Print(" %.8f", pitchRate);
            //UART_Print(" %.4f", pitchPid.rate_calc);
            
            //UART_Print(" rollP %.4f", rollPid.output);
            //UART_Print(" pitchP %.4f", pitchPid.output);
            //UART_Print(" yawP %.8f", yawPid.output);
            //UART_Print(" altP %.4f", altitudePid.output);
            
            
            
            //UART_Print(" esc1 %.4f", esc1);
            //UART_Print(" esc2 %.4f", esc2);
            //UART_Print(" esc3 %.4f", esc3);
            //UART_Print(" esc4 %.4f", esc4);
            
            //input_pwm_ch1 = TIM1->CCR1;
            //input_pwm_ch2 = TIM1->CCR2;
            //input_pwm = (float)(100 * (input_pwm_ch2 / input_pwm_ch1));
            //UART_Print(" a1: %.4f", a1);
            //UART_Print(" a2: %.4f", a2);
            //UART_Print(" a3: %.4f", a3);
            
            // Read joystick
            //UART_Print(" hj: %.4f", Joystick_ReadDuty(&yawJoystick));
            //UART_Print(" aj: %.4f", Joystick_ReadDuty(&pitchJoystick));
            /*UART_Print(" tim4: %.4f", Joystick_ReadDuty(&rollJoystick));
            UART_Print(" tim5: %.4f", Joystick_ReadDuty(&thrustJoystick));
            UART_Print(" tim9: %.4f", Joystick_ReadDuty(&switchlJoystick));
            UART_Print(" tim12: %.4f", Joystick_ReadDuty(&switchrJoystick));
            */
            UART_Print(" a: %d", arm);
            UART_Print(" h: %.4f", hover);
            //UART_Print(" p: %.6f", yawPid.k_p);
            //UART_Print(" d: %.6f", yawPid.k_d);
            //UART_Print(" p: %.6f", yawPid.k_p);
            
            UART_Print("\n\r");
        }
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    }
    /* USER CODE END Telemetry_StartTask */
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
