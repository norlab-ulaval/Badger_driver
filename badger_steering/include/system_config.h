#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#include "stm32f3xx_hal.h"

/* Exported peripheral handles */
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;

/* Exported functions */
void SystemClock_Config(void);
void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);

#endif // __SYSTEM_CONFIG_H
