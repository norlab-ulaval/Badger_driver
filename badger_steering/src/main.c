// Application entry and realtime logic only
#include <stdio.h>
#include "system_config.h"

volatile uint16_t adc_value = 0;
uint8_t txbuf[4];   // [0xAA | low_byte | high_nibble | XOR_checksum]


int main(void)
{
    HAL_Init();

    SystemClock_Config();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM6_Init();

    HAL_TIM_Base_Start(&htim6);

    HAL_ADC_Start_IT(&hadc2);

    while (1)
    {
        // I'M ALIVE !!!!
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // ADC conversions are triggered automatically by TIM6 TRGO (hardware trigger).
    // We just need to clear the register
    (void)htim;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Checking if ADC has a value to extract
    if (hadc->Instance == ADC2)
    {
        adc_value = HAL_ADC_GetValue(&hadc2);

        // 4-byte framed packet: [start | low | high_nibble | checksum]
        txbuf[0] = 0xAA;                          // start marker
        txbuf[1] = adc_value & 0xFF;              // low byte
        txbuf[2] = (adc_value >> 8) & 0x0F;       // high nibble (bits 8-11)
        txbuf[3] = txbuf[1] ^ txbuf[2];           // XOR checksum

        // Sending to UART bus / USB
        HAL_UART_Transmit(&huart2, txbuf, 4, 1);
    }
}