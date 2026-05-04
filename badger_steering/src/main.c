// Application entry and logic only. Hardware/system configuration moved to system_config module.
#include <stdio.h>
#include "system_config.h"




// Redirect printf to UART
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART2_UART_Init();
    MX_ADC1_Init();

    uint32_t rawValue = 0;

    while (1) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            rawValue = HAL_ADC_GetValue(&hadc1);
            printf("Steering amount: %lu\r\n", rawValue);
        }
        HAL_Delay(500); // Read twice per second
    }
}