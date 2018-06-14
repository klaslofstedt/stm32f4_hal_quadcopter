#include "stm32f4xx_hal.h"
#include "uart_print.h"
#include "hardware.h"
#include "string.h"

#include <stdarg.h>
#include <stdlib.h>

//SemaphoreHandle_t g_uart_mutex;

void UART_Print(const char *format, ...)
{
    //xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
    
    va_list list;
    va_start(list, format);
    int len = vsnprintf(0, 0, format, list);
    char *s;
    s = (char *)malloc(len + 1);
    vsprintf(s, format, list);

    HAL_UART_Transmit(&huart2, (uint8_t *)s, len, 100);
    free(s);
    va_end(list);
    
    //xSemaphoreGive(g_uart_mutex);
}