#ifndef HARDWARE_H
#define HARDWARE_H

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_I2C3_Init(void);
void MX_SPI1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM9_Init(void);
void MX_TIM12_Init(void);
void MX_TIM13_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

#endif