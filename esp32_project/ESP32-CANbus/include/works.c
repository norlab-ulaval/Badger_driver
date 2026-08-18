// #include <string.h>
// #include <stdlib.h>
// #include <stdatomic.h>
// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "driver/gpio.h"
// #include "driver/twai.h"
// #include "driver/pulse_cnt.h" 
// #include "esp_log.h"
// #include "custom.h"

// // Shared state between cores and interruptions

// static QueueHandle_t g_encoder_queue = NULL;

// static _Atomic int32_t g_pwm_duty = 0;

// // Core 1 – Encoder + PWM (APP_CPU) see datasheet of the esp 32 processor

// static pcnt_unit_handle_t encoder_hw_global = NULL;
// static volatile int64_t g_last_z_trigger_us = 0;
// static _Atomic bool g_z_index_flag = false;

// // Tick counter placed in internal RAM 

// static void IRAM_ATTR z_isr_handler(void *arg) {
//     int64_t now = esp_timer_get_time();
//     if ((now - g_last_z_trigger_us) < ENCODER_Z_DEBOUNCE_US) {
//         return;
//     }
//     g_last_z_trigger_us = now;
//     atomic_store(&g_z_index_flag, true);
// }

// static void task_encoder_pwm(void *arg)
// {
//     int64_t accumulated_count = 0;
//     int64_t current_total     = 0;
//     int64_t last_total        = 0;
//     int64_t last_accumulated  = 0;
//     float last_valid_rpm      = 0.0f;

//     bool    locked_reverse  = true;   
//     int32_t zero_rpm_streak = 0;

//     TickType_t last_wake = xTaskGetTickCount();

//     while (1) {
//         bool z_reset_this_cycle = false;

//         // This loop collect the encoder tick and applies anti vibration
//         // correction to make sure that the measured rpm's are not impacted by the shaft's imperfet rotation
//         // An hysteresis to make sure the directio n does'nt switch when it isn't actually switching
//         // Digital filtering
//         // anti dittering in the z-ISR  

//         if (atomic_exchange(&g_z_index_flag, false)) {
//             encoder_reset_count(encoder_hw_global);
//             last_total = 0;
//             z_reset_this_cycle = true;
//         } else {
//             current_total = encoder_get_total_count(encoder_hw_global);
//             int32_t delta = (int32_t)(current_total - last_total);

//             if (abs(delta) < ENCODER_DITHER_THRESHOLD) {
//                 delta = 0;
//             }

//             last_total = current_total;
//             accumulated_count += delta;
//         }

//         int64_t velocity = (accumulated_count - last_accumulated) * 100;
//         last_accumulated = accumulated_count;

//         float rpm;
//         if (z_reset_this_cycle) {
//             rpm = last_valid_rpm;
//         } else {
//             rpm = (float)velocity * 60.0f / (float)ENCODER_TICKS_PER_REV;
//             last_valid_rpm = rpm;
//         }

//         encoder_msg_t enc = { .rpm = rpm };
//         xQueueOverwrite(g_encoder_queue, &enc);

//         if (rpm == 0.0f) {
//             if (zero_rpm_streak < INT32_MAX) {
//                 zero_rpm_streak++;
//             }
//         } else {
//             zero_rpm_streak = 0;
//         }

//         int32_t  commanded = atomic_load(&g_pwm_duty);
//         uint32_t magnitude = (uint32_t)fabsf((float)commanded);

//         if (commanded == 0) {
//             pwm_set_duty(0);
//         } else {
//             bool desired_reverse = commanded < 0;

//             if (desired_reverse == locked_reverse) {
//                 pwm_set_duty(magnitude);
//             } else if (zero_rpm_streak >= ENCODER_ZERO_RPM_HOLD) {
//                 locked_reverse = desired_reverse;
//                 pwm_set_direction(locked_reverse);
//                 pwm_set_duty(magnitude);
//             } else {
//                 pwm_set_duty(0);
//             }
//         }

//         vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10)); // 100 Hz
//     }
// }

// // ---------------------------------------------------------------------------
// // Core 0 – CAN bus (PRO_CPU)
// // ---------------------------------------------------------------------------
// static void task_can(void *arg)
// {
//     twai_message_t tx_msg = {
//         .identifier       = CAN_ID_ENCODER_TX,
//         .data_length_code = 4,
//         .flags            = TWAI_MSG_FLAG_NONE,
//     };
//     twai_message_t rx_msg;
//     uint8_t        tx_divider = 0;
//     uint8_t        pwm_watchdog = 0;

//     TickType_t last_wake = xTaskGetTickCount();

//     while (1) {
//         if (twai_receive(&rx_msg, 0) == ESP_OK) {
//             if (rx_msg.identifier == CAN_ID_PWM_CMD && rx_msg.data_length_code >= 2) {
//                 int16_t raw = 0;
//                 memcpy(&raw, rx_msg.data, sizeof(raw));

//                 int32_t clamped = (int32_t)raw;
//                 if (clamped > (int32_t)PWM_MAX_DUTY)  clamped = (int32_t)PWM_MAX_DUTY;
//                 if (clamped < -(int32_t)PWM_MAX_DUTY) clamped = -(int32_t)PWM_MAX_DUTY;

//                 atomic_store(&g_pwm_duty, clamped);
//                 pwm_watchdog = 0;
//             }
//         }

//         if(++pwm_watchdog >= 10){
//             atomic_store(&g_pwm_duty, 0);
//             pwm_watchdog = 0;
//         }

//         if (++tx_divider >= 4) {
//             tx_divider = 0;
//             encoder_msg_t enc;
//             if (xQueuePeek(g_encoder_queue, &enc, 0) == pdTRUE) {
//                 memcpy(tx_msg.data, &enc.rpm, sizeof(float));
//                 twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
//             }
//         }

//         vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
//     }
// }


// void app_main(void)
// {
//     // CAN bus hardware init
//     gpio_reset_pin(CAN_TX_IO);
//     gpio_reset_pin(CAN_RX_IO);

//     twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_IO, CAN_RX_IO, TWAI_MODE_NORMAL);
//     twai_filter_config_t f_config = {
//         .acceptance_code = 0x20000000,
//         .acceptance_mask = 0x01FFFFFF,
//         .single_filter   = true,
//     };
//     twai_timing_config_t t_config = {
//         .brp             = 20,
//         .prop_seg        = 0,
//         .tseg_1          = 13,
//         .tseg_2          = 2,
//         .sjw             = 1,
//         .triple_sampling = false,
//     };

//     ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
//     ESP_ERROR_CHECK(twai_start());

//     // Encoder hardware init.
//     encoder_hw_global = init_quadrature_encoder();

//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(ENCODER_Z_GPIO, z_isr_handler, NULL);

//     pwm_init();

//     g_encoder_queue = xQueueCreate(1, sizeof(encoder_msg_t));

//     xTaskCreatePinnedToCore(task_can, "task_can", 4096, NULL, 5, NULL, 0);
//     xTaskCreatePinnedToCore(task_encoder_pwm, "task_enc_pwm", 4096, NULL, 5, NULL, 1);
// }