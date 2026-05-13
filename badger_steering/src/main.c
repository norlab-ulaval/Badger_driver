// Application entry and realtime logic only
#include <stdio.h>
#include "system_config.h"

// Both ADC values captured at 1 kHz via TIM6 TRGO hardware trigger
volatile uint16_t adc1_value = 0;
volatile uint16_t adc2_value = 0;

// Ready flags set inside the ISR, cleared after UART send
volatile uint8_t adc1_ready = 0;
volatile uint8_t adc2_ready = 0;

// 6-byte framed packet: [0xAA | adc1_low | adc1_high_nibble | adc2_low | adc2_high_nibble | XOR_checksum]
uint8_t txbuf[6];


int main(void)
{
    HAL_Init();

    SystemClock_Config();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    // ADC2 activates the shared ADC1_2 interrupt, so configure it last
    MX_ADC2_Init();

    // STM32F3 requires calibration before first use — without this, readings are garbage
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    MX_TIM6_Init();

    // Start timer: TIM6 TRGO fires at 1 kHz and triggers both ADC conversions
    HAL_TIM_Base_Start_IT(&htim6);

    // Both ADCs in interrupt mode — conversions started by hardware (TIM6 TRGO)
    HAL_ADC_Start_IT(&hadc1);
    HAL_ADC_Start_IT(&hadc2);

    while (1)
    {
        // When both ADC samples are ready, build and send the combined UART packet

        // I'm alive !!!!!!!!
        if (adc1_ready && adc2_ready)
        {
            adc1_ready = 0;
            adc2_ready = 0;
            Create_Tx_buffer(adc1_value, adc2_value, txbuf);
            HAL_UART_Transmit(&huart2, txbuf, 6, 2);
        }
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // ADC conversions are triggered automatically by TIM6 TRGO (hardware trigger).
    // Nothing to do here — the ADC ISR handles the data.
    (void)htim;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc1_value = HAL_ADC_GetValue(&hadc1);
        adc1_ready = 1;
    }
    else if (hadc->Instance == ADC2)
    {
        adc2_value = HAL_ADC_GetValue(&hadc2);
        adc2_ready = 1;
    }
}