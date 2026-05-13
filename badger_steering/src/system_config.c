#include "system_config.h"

// Hardware identifier INIT
UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim6;

// oscillator setup basic clock speed used here
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;

    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}


// Configuring the A0 pin(physical layout) or PA0 (chip layout) for analog readings
// The pin is then connected to ADC1 
// and a timer interrupt(tim6) assures constant frequency sampling

void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler       = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution           = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign            = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode         = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection         = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait     = DISABLE;
    hadc1.Init.ContinuousConvMode   = DISABLE;
    hadc1.Init.NbrOfConversion      = 1;
    hadc1.Init.DiscontinuousConvMode= DISABLE;

    // Triggered by the Tim6 configed lower 
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T6_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;

    // Instance of ADC 2 configured with the mentionned parameters above
    HAL_ADC_Init(&hadc1);
    // Refer to photo in ReadMe
    sConfig.Channel      = ADC_CHANNEL_1;   
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    
}



// Configuring the A5 pin(physical layout) or PA6(Chip layout) for analog readings
// The pin is then connected to ADC2 
// and a timer interrupt(tim6) assures constant frequency sampling

void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler       = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution           = ADC_RESOLUTION_12B;
    hadc2.Init.DataAlign            = ADC_DATAALIGN_RIGHT;
    hadc2.Init.ScanConvMode         = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection         = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait     = DISABLE;
    hadc2.Init.ContinuousConvMode   = DISABLE;
    hadc2.Init.NbrOfConversion      = 1;
    hadc2.Init.DiscontinuousConvMode= DISABLE;

    // Triggered by the Tim6 configed lower 
    hadc2.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T6_TRGO;
    hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;

    // Instance of ADC 2 configured with the mentionned parameters above
    HAL_ADC_Init(&hadc2);
    // Refer to photo in ReadMe
    sConfig.Channel      = ADC_CHANNEL_3;   
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    
    // Enabling Interrupts
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}


// Configuring the UART port to do USB serial communication

void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;

    // UART config (basic)

    huart2.Init.BaudRate   = 921600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart2);
}


// Defining the timing / frequency of the interrupts
void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    // Frequency is computed with f = (f_CLK) / ((PSC+1)*(period + 1))
    htim6.Init.Prescaler = 63;
    htim6.Init.Period    = 999;
    // 1khz setting
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&htim6);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}



// Builds a 6-byte framed packet combining both ADC channels:
// [0xAA | adc1_low | adc1_high_nibble | adc2_low | adc2_high_nibble | XOR_checksum]
void Create_Tx_buffer(uint16_t adc1, uint16_t adc2, uint8_t *buffer)
{
    buffer[0] = 0xAA;
    buffer[1] = (uint8_t)(adc1 & 0xFF);
    buffer[2] = (uint8_t)((adc1 >> 8) & 0x0F);
    buffer[3] = (uint8_t)(adc2 & 0xFF);
    buffer[4] = (uint8_t)((adc2 >> 8) & 0x0F);
    buffer[5] = buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4];
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

void ADC1_2_IRQHandler(void)
{
    // Both ADC1 and ADC2 share this IRQ — service both so their callbacks fire
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
}