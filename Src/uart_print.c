#include "stm32f4xx_hal.h"
#include "uart_print.h"
#include "hardware.h"
#include "string.h"

#include <stdarg.h>
#include <stdlib.h>

//SemaphoreHandle_t g_uart_mutex;

//void UART_Print(UART_HandleTypeDef *huart, char _out[])
/*void UART_Print(char _out[])
{
    //HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
    HAL_UART_Transmit(&huart2, (uint8_t *) _out, strlen(_out), 10);
}*/




void UART_Print(const char *format, ...)
{
    //xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
    
    va_list list;
    va_start(list, format);
    int len = vsnprintf(0, 0, format, list);
    char *s;
    s = (char *)malloc(len + 1);
    vsprintf(s, format, list);
    //USART_puts(s);
    HAL_UART_Transmit(&huart2, (uint8_t *) s, len, 10);
    free(s);
    va_end(list);
    
    
    
    //xSemaphoreGive(g_uart_mutex);
}