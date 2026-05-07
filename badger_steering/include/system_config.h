#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#include "stm32f3xx_hal.h"


// Hardware handler librairies or functions
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim6;

// Hardware config declarations
void SystemClock_Config(void);

void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
void MX_TIM6_Init(void);


// Interruption handling
void SysTick_Handler(void);

void TIM6_DAC_IRQHandler(void);
void ADC1_2_IRQHandler(void);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);


#endif /* __SYSTEM_CONFIG_H */