#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#include "stm32f3xx_hal.h"


// Hardware handler librairies or functions
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;

// Hardware config declarations
void SystemClock_Config(void);

void MX_USART2_UART_Init(void);
// ADC 1 for azimut angle
void MX_ADC1_Init(void);
// ADC 2 for longitude angle
void MX_ADC2_Init(void);
// Frequency setting
void MX_TIM6_Init(void);


// Interruption handling
void SysTick_Handler(void);

void TIM6_DAC_IRQHandler(void);
void ADC1_2_IRQHandler(void);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

// Builds a 6-byte combined packet: [0xAA | adc1_low | adc1_high | adc2_low | adc2_high | checksum]
void Create_Tx_buffer(uint16_t adc1, uint16_t adc2, uint8_t *buffer);


#endif /* __SYSTEM_CONFIG_H */