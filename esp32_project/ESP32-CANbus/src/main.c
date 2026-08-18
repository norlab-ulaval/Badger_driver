#include "custom.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG_CAN = "CAN_DRIVER";

// Inter-task and inter-core state
static QueueHandle_t g_encoder_queue = NULL;

static _Atomic float g_cmd_velocity_mps = 0.0f;

static _Atomic bool g_estop_active = true;
static _Atomic bool g_recovery_mode = false;
static _Atomic bool g_open_loop_mode_enabled = false;

static _Atomic uint8_t g_vehicle_soc = 0;
static _Atomic int32_t g_recovery_steering_ticks = 0;
static _Atomic bool g_heartbeat_seen = false;

static _Atomic uint8_t g_connection_vsc;

// Encoder hardware unit
static pcnt_unit_handle_t encoder_hw_global = NULL;
static volatile int64_t g_last_z_trigger_us = 0;
static _Atomic bool g_z_index_flag = false;

static volatile uint32_t g_debug_pid_reset_count = 0;

static void IRAM_ATTR z_isr_handler(void *arg) {
  int64_t now = esp_timer_get_time();
  if ((now - g_last_z_trigger_us) < ENCODER_Z_DEBOUNCE_US) {
    return;
  }
  g_last_z_trigger_us = now;
  atomic_store(&g_z_index_flag, true);
}

// ---------------------------------------------------------------------------
// Core 1 – Encoder debugging & Closed-Loop PWM Task
// ---------------------------------------------------------------------------
static void task_hardware_loop(void *arg) {
  encoder_track_t enc_track = {0};

  int64_t zero_rpm_since_us = 0;

  bool filt_initialized = false;
  float rpm_filt = 0.0f;

  // float ramped_target_mps = 0.0f; // slew-rate limiter disabled for testing
  float prev_nonzero_target_sign = 0.0f;

  const int64_t ZERO_RPM_HOLD_US = (int64_t)ENCODER_ZERO_RPM_HOLD * 10000LL;

  pid_ctrl_t vel_pid = {
      .kp = PID_KP,
      .ki = PID_KI,
      .kd = PID_KD,
      .integral = 0.0f,
      .prev_measured = 0.0f,
      .prev_measured_valid = false,
      .d_filt = 0.0f,
      .out_filt = 0.0f,
      .out_filt_initialized = false,
      .output_min = -(float)PWM_MAX_DUTY,
      .output_max = (float)PWM_MAX_DUTY,
      .locked_reverse = true,
  };

  pwm_drive(&vel_pid, 0, true);

  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    bool z_index_event = atomic_exchange(&g_z_index_flag, false);
    encoder_msg_t enc_raw =
        encoder_update(encoder_hw_global, &enc_track, z_index_event);

    // Input filtering can be tuned via custom.h used to dampen the encoder's
    // discretization smoothing out the ticks

    if (!filt_initialized) {
      rpm_filt = enc_raw.rpm;
      filt_initialized = true;
    } else {
      rpm_filt += VEL_FILTER_ALPHA * (enc_raw.rpm - rpm_filt);
    }

    encoder_msg_t enc = enc_raw;
    enc.rpm = rpm_filt;
    xQueueOverwrite(g_encoder_queue, &enc);

    // Stamping the encoder measurements (various utilies lol)

    int64_t now_us = esp_timer_get_time();
    if (fabsf(rpm_filt) < FORCE_ZERO) {
      if (zero_rpm_since_us == 0) {
        zero_rpm_since_us = now_us;
      }
    } else {
      zero_rpm_since_us = 0;
    }
    bool zero_rpm_hold_elapsed =
        (zero_rpm_since_us != 0) &&
        ((now_us - zero_rpm_since_us) >= ZERO_RPM_HOLD_US);

    // E-Stop hard override (FORT VSC heartbeat estop OR stale command watchdog)
    if (atomic_load(&g_estop_active)) {
      pwm_drive(&vel_pid, 0, true);
      pid_reset(&vel_pid);
      filt_initialized = false;
      // ramped_target_mps = 0.0f; // slew-rate limiter disabled for testing
      g_debug_pid_reset_count++;
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_LOOP_PERIOD_MS));
      continue;
    }

    float target_mps_raw = atomic_load(&g_cmd_velocity_mps);
    float measured_mps = rpm_filt * RPM_TO_MPS;

    // Slew-rate limiter (disabled for testing)
    // Usually it is used to limit acceleration in large error condition
    // {
    //   float max_step = TARGET_SLEW_MAX_MPS_PER_S * PID_DT_S;
    //   float diff = target_mps_raw - ramped_target_mps;
    //   if (diff > max_step) {
    //     ramped_target_mps += max_step;
    //   } else if (diff < -max_step) {
    //     ramped_target_mps -= max_step;
    //   } else {
    //     ramped_target_mps = target_mps_raw;
    //   }
    // }
    // float target_mps = ramped_target_mps;
    float target_mps = target_mps_raw;

    // Direction reversal detection we have to force saturate the command to 0
    // because the DC motor cannot reverse it's direction during movement
    {
      float current_sign = (target_mps > 0.0f)   ? 1.0f
                           : (target_mps < 0.0f) ? -1.0f
                                                 : 0.0f;
      if (current_sign != 0.0f) {
        if (prev_nonzero_target_sign != 0.0f &&
            current_sign != prev_nonzero_target_sign) {
          pid_reset(&vel_pid);
        }
        prev_nonzero_target_sign = current_sign;
      }
    }

    if (target_mps > 0.0f) {
      vel_pid.output_min = 0.0f;
      vel_pid.output_max = (float)PWM_MAX_DUTY;
    } else if (target_mps < 0.0f) {
      vel_pid.output_min = -(float)PWM_MAX_DUTY;
      vel_pid.output_max = 0.0f;
    } else {
      vel_pid.output_min = -(float)PWM_MAX_DUTY;
      vel_pid.output_max = (float)PWM_MAX_DUTY;
    }

    float integral_before = vel_pid.integral;
    float pid_out = pid_update(&vel_pid, target_mps, measured_mps, PID_DT_S);

    int32_t commanded = (int32_t)lroundf(pid_out);

    bool held_by_direction_lock =
        pwm_drive(&vel_pid, commanded, zero_rpm_hold_elapsed);

    if (held_by_direction_lock) {
      vel_pid.integral = integral_before;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_LOOP_PERIOD_MS));
  }
}

// Utility function to deal with the latching buttons of the fort remote

static bool latch_toggle_update(twai_message_t rx_msg, uint32_t byte,
                                uint32_t bit_offset, uint32_t mask,
                                uint32_t set_value, int *prev_value,
                                _Atomic bool *mode_flag) {
  int latch_value = can_decode(rx_msg, byte, bit_offset, mask);

  bool toggled = false;
  if (*prev_value >= 0 && *prev_value != (int)set_value &&
      latch_value == (int)set_value) {
    atomic_store(mode_flag, !atomic_load(mode_flag));
    toggled = true;
  }
  *prev_value = latch_value;
  return toggled;
}

// ---------------------------------------------------------------------------
// Core 0 – CAN Bus Protocol Task
// ---------------------------------------------------------------------------
static void task_can(void *arg) {

  twai_message_t rx_msg;
  uint8_t tx_divider = 0;
  uint8_t pwm_watchdog = 0;
  uint8_t steering_tx_divider = 0;
  uint16_t fort_display_divider = 0;

  int prev_estop_button = -1;
  int prev_open_loop_button = -1;
  bool roboteq_estop_latched = true;

  // g_estop_active fuses two independent stop conditions; each is tracked
  // locally so a heartbeat update can't clobber a stale-command stop (or
  // vice versa) when the combined flag is republished.
  bool hw_estop_active = true;
  bool cmd_watchdog_expired = true;

  static int32_t open_loop_target = 0;

  TickType_t last_wake = xTaskGetTickCount();

  fort_joy_state right_x = {.status_byte = RIGHT_JOY_X_STATUS_BYTE,
                            .neutral_status = 0,
                            .positive_status = 0,
                            .negative_status = 0,
                            .magnitude = 0.0f,
                            .neutral_shift = RIGHT_JOY_X_NEUTRAL_SHIFT,
                            .positive_shift = RIGHT_JOY_X_POSITIVE_SHIFT,
                            .negative_shift = RIGHT_JOY_X_NEGATIVE_SHIFT,
                            .status_mask = RIGHT_JOY_X_STATUS_MASK,
                            .mag_shift = RIGHT_JOY_X_MAG_LOW_SHIFT,
                            .mag_byte = RIGHT_JOY_X_MAG_HIGH_BYTE};

  fort_joy_state right_y = {.status_byte = RIGHT_JOY_Y_STATUS_BYTE,
                            .neutral_status = 0,
                            .positive_status = 0,
                            .negative_status = 0,
                            .magnitude = 0.0f,
                            .neutral_shift = RIGHT_JOY_Y_NEUTRAL_SHIFT,
                            .positive_shift = RIGHT_JOY_Y_POSITIVE_SHIFT,
                            .negative_shift = RIGHT_JOY_Y_NEGATIVE_SHIFT,
                            .status_mask = RIGHT_JOY_Y_STATUS_MASK,
                            .mag_shift = RIGHT_JOY_Y_MAG_LOW_SHIFT,
                            .mag_byte = RIGHT_JOY_Y_MAG_HIGH_BYTE};

  // -----------------------------------------------------------------
  // ROBOTEQ 2460s CANOpen standard SDO messages extracted from the
  // .eds (electronic datasheet) as well as roboteq's user manual
  // -----------------------------------------------------------------

  // Steering message (From roboteq MDC 2460s User Manual and .eds) !CANGO
  twai_message_t steering_msg = {
      .identifier = ROBOTEQ_SDO_TX_ID,
      .data_length_code = 8,
      .flags = TWAI_MSG_FLAG_NONE,
  };

  steering_msg.data[0] = (uint8_t)ROBOTEQ_SDO_CS_EXPEDITED_4B;
  steering_msg.data[1] = (uint8_t)(ROBOTEQ_ART_CANOPEN_INDEX & 0xFFu);
  steering_msg.data[2] = (uint8_t)((ROBOTEQ_ART_CANOPEN_INDEX >> 8) & 0xFFu);
  steering_msg.data[3] = (uint8_t)ROBOTEQ_ART_CANOPEN_SUBINDEX;

  // Estop (From roboteq MDC 2460s User Manual) --> !EX in the serial terminal
  twai_message_t estop_message = {
      .identifier = ROBOTEQ_SDO_TX_ID,
      .data_length_code = 8,
      .flags = TWAI_MSG_FLAG_NONE,
  };
  estop_message.data[0] = (uint8_t)ROBOTEQ_SDO_CS_EXPEDITED_1B;
  estop_message.data[1] = (uint8_t)(ROBOTEQ_ESTOP_CANOPEN_INDEX & 0xFFu);
  estop_message.data[2] = (uint8_t)((ROBOTEQ_ESTOP_CANOPEN_INDEX >> 8) & 0xFFu);
  estop_message.data[3] = (uint8_t)ROBOTEQ_ESTOP_CANOPEN_SUBINDEX;
  estop_message.data[4] = 1u;

  // Motor go means release estop (From roboteq MDC 2460s User Manual) --> !MG
  // in the serial terminal
  twai_message_t motor_go_message = {
      .identifier = ROBOTEQ_SDO_TX_ID,
      .data_length_code = 8,
      .flags = TWAI_MSG_FLAG_NONE,
  };
  motor_go_message.data[0] = (uint8_t)ROBOTEQ_SDO_CS_EXPEDITED_1B;
  motor_go_message.data[1] = (uint8_t)(ROBOTEQ_RELEASE_CANOPEN_INDEX & 0xFFu);
  motor_go_message.data[2] =
      (uint8_t)((ROBOTEQ_RELEASE_CANOPEN_INDEX >> 8) & 0xFFu);
  motor_go_message.data[3] = (uint8_t)ROBOTEQ_ESTOP_CANOPEN_SUBINDEX;
  motor_go_message.data[4] = 1u;

  // --------------------------------------------------
  // MAIN LOOP FOR CAN BUS MANAGEMENT
  // --------------------------------------------------
  while (1) {
    // 1. TWAI State Monitoring & Recovery
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
      if (status.state == TWAI_STATE_BUS_OFF) {

        ESP_LOGE(TAG_CAN, "TWAI Bus-Off detected! Auto-recovering...");
        twai_initiate_recovery();
      } else if (status.state == TWAI_STATE_STOPPED) {
        ESP_LOGI(TAG_CAN, "TWAI recovered. Restarting...");
        twai_start();
      }
    }
    // -------------------------------------------------
    // 2. Process CAN bus RX Buffer
    // -------------------------------------------------
    while (twai_receive(&rx_msg, 0) == ESP_OK) {

      // A. FORT VSC Heartbeat handler
      if (rx_msg.identifier == CAN_ID_VSC_HEARTBEAT &&
          (rx_msg.flags & TWAI_MSG_FLAG_EXTD) &&
          rx_msg.data_length_code >= (ESTOP_BYTE_OFFSET + 4)) {
        atomic_store(&g_heartbeat_seen, true);

        uint32_t estop = 0;
        memcpy(&estop, &rx_msg.data[ESTOP_BYTE_OFFSET], sizeof(uint32_t));
        hw_estop_active = (estop != 0);
        atomic_store(&g_estop_active, hw_estop_active || cmd_watchdog_expired);

        if (hw_estop_active) {
          atomic_store(&g_recovery_mode, false);
          twai_transmit(&estop_message, pdMS_TO_TICKS(10));
          roboteq_estop_latched = true;
        }
      }
      // B. FORT VSC Control Messsages
      else if (rx_msg.identifier == CAN_ID_VSC_RIGHT_JOYSTICK_BUTTONS &&
               (rx_msg.flags & TWAI_MSG_FLAG_EXTD) &&
               rx_msg.data_length_code >= (RECOVERY_LATCH_BYTE + 1)) {

        // Toggle Recovery Mode (Button4)
        if (latch_toggle_update(rx_msg, RECOVERY_LATCH_BYTE,
                                RECOVERY_LATCH_BIT_OFFSET, RECOVERY_LATCH_MASK,
                                LATCH_SET_VALUE, &prev_estop_button,
                                &g_recovery_mode)) {
          if (atomic_load(&g_recovery_mode) && roboteq_estop_latched &&
              !hw_estop_active) {
            twai_transmit(&motor_go_message, pdMS_TO_TICKS(10));
            roboteq_estop_latched = false;
          }
        }

        // Toggle Open Loop Mode (Button1)
        if (rx_msg.data_length_code >= (OPEN_LOOP_LATCH_BYTE + 1)) {
          latch_toggle_update(rx_msg, OPEN_LOOP_LATCH_BYTE,
                              OPEN_LOOP_LATCH_BIT_OFFSET, OPEN_LOOP_LATCH_MASK,
                              LATCH_SET_VALUE, &prev_open_loop_button,
                              &g_open_loop_mode_enabled);
        }

        // Processes the FORT joystick in recovery mode

        if (atomic_load(&g_recovery_mode)) {
          // Process Right Y axis speed target during recovery mode
          if (rx_msg.data_length_code >= (RIGHT_JOY_Y_MAG_HIGH_BYTE + 1)) {

            fort_joystick_read(rx_msg, &right_y);

            float velocity_mps = 0.0f;
            if (right_y.neutral_status != JOYSTICK_STATUS_SET &&
                right_y.magnitude != 0) {
              if (right_y.negative_status == JOYSTICK_STATUS_SET) {
                velocity_mps = -right_y.magnitude * JOYSTICK_MAX_SPEED_MPS;
              } else if (right_y.positive_status == JOYSTICK_STATUS_SET) {
                velocity_mps = right_y.magnitude * JOYSTICK_MAX_SPEED_MPS;
              }
            }
            atomic_store(&g_cmd_velocity_mps, velocity_mps);
            cmd_watchdog_expired = false;
            atomic_store(&g_estop_active,
                         hw_estop_active || cmd_watchdog_expired);
            pwm_watchdog = 0;
          }

          // Process Right X axis steering target
          if (rx_msg.data_length_code >= (RIGHT_JOY_X_MAG_HIGH_BYTE + 1)) {

            fort_joystick_read(rx_msg, &right_x);

            if (atomic_load(&g_open_loop_mode_enabled)) {
              // Open loop steering integrator
              if (right_x.neutral_status != JOYSTICK_STATUS_SET &&
                  right_x.magnitude != 0) {

                int32_t step = (int32_t)lroundf(right_x.magnitude *
                                                (float)OPEN_LOOP_TICKS);

                if (right_x.negative_status == JOYSTICK_STATUS_SET) {
                  open_loop_target =
                      (open_loop_target + step > ROBOTEQ_MAX_TICKS)
                          ? ROBOTEQ_MAX_TICKS
                          : open_loop_target + step;
                } else if (right_x.positive_status == JOYSTICK_STATUS_SET) {
                  open_loop_target =
                      (open_loop_target - step < -ROBOTEQ_MAX_TICKS)
                          ? -ROBOTEQ_MAX_TICKS
                          : open_loop_target - step;
                }
              }
              atomic_store(&g_recovery_steering_ticks, open_loop_target);
            } else {
              // Closed loop steering proportional
              int32_t steering_ticks = 0;
              if (right_x.neutral_status != JOYSTICK_STATUS_SET &&
                  right_x.magnitude != 0.0f) {

                if (right_x.negative_status == JOYSTICK_STATUS_SET) {
                  steering_ticks = (int32_t)lroundf(
                      right_x.magnitude * (float)ROBOTEQ_STEERING_MAX_TICKS);
                } else if (right_x.positive_status == JOYSTICK_STATUS_SET) {
                  steering_ticks = -(int32_t)lroundf(
                      right_x.magnitude * (float)ROBOTEQ_STEERING_MAX_TICKS);
                }
              }
              open_loop_target = steering_ticks;
              atomic_store(&g_recovery_steering_ticks, steering_ticks);
            }
          }
        }
      }
      // C. ROS Velocity Command Frame
      else if (!atomic_load(&g_recovery_mode) &&
               rx_msg.identifier == CAN_ID_SPEED_CMD &&
               rx_msg.data_length_code >= 4) {
        float velocity_mps = 0.0f;
        memcpy(&velocity_mps, rx_msg.data, sizeof(float));

        atomic_store(&g_cmd_velocity_mps, velocity_mps);
        cmd_watchdog_expired = false;
        atomic_store(&g_estop_active, hw_estop_active || cmd_watchdog_expired);
        pwm_watchdog = 0;
      }
      // D. MTT BMS Core Status (0x602 / ID 1538): Extract StateOfCharge
      else if (rx_msg.identifier == CAN_ID_MTT_BMS_CORE_STATUS_602 &&
               !(rx_msg.flags & TWAI_MSG_FLAG_EXTD) &&
               rx_msg.data_length_code >= 1) {
        uint8_t soc = rx_msg.data[0];
        atomic_store(&g_vehicle_soc, soc);
      }
      // E. Remote to VSC connection status
      else if (rx_msg.identifier == CAN_ID_VSC_CONNECTION &&
               (rx_msg.flags & TWAI_MSG_FLAG_EXTD) &&
               rx_msg.data_length_code >= 3) {

        uint8_t conn_stength_vsc = rx_msg.data[2];
        atomic_store(&g_connection_vsc, conn_stength_vsc);
      }
    }

    // 3. CAN bus Transmission
    twai_message_t encoder_msg = {
        .identifier = CAN_ID_ENCODER_TX,
        .data_length_code = 4,
        .flags = TWAI_MSG_FLAG_NONE,
    };

    // A. Transmit Encoder RPM Data (25 Hz)
    if (++tx_divider >= 4) {
      tx_divider = 0;
      encoder_msg_t enc;
      if (xQueuePeek(g_encoder_queue, &enc, 0) == pdTRUE) {

        memcpy(encoder_msg.data, &enc.rpm, sizeof(float));

        twai_transmit(&encoder_msg, pdMS_TO_TICKS(10));
      }
    }

    // B. Transmit Steering SDO while in Recovery Mode
    if (atomic_load(&g_recovery_mode)) {
      if (++steering_tx_divider >= STEERING_SEND_DIVIDER) {
        steering_tx_divider = 0;
        int32_t steering_ticks = atomic_load(&g_recovery_steering_ticks);

        memcpy(&steering_msg.data[4], &steering_ticks, sizeof(int32_t));

        twai_transmit(&steering_msg, pdMS_TO_TICKS(10));
      }
    } else {
      steering_tx_divider = 0;
    }

    // C. Transmit Telemetry to FORT Remote Display (2 Hz / 500ms)
    if (++fort_display_divider >= 50) {
      fort_display_divider = 0;

      uint8_t soc = atomic_load(&g_vehicle_soc);
      uint8_t conn_stength = atomic_load(&g_connection_vsc);

      vsc_send_feedback_value(1, (int32_t)soc);
      vsc_send_feedback_value(2, (int32_t)conn_stength);

      char soc_str[7];
      snprintf(soc_str, sizeof(soc_str), "%u%%", (unsigned int)soc);
      vsc_send_feedback_string_segment(3, 0, soc_str);
    }

    if (++pwm_watchdog >= PWM_WATCHDOG_POLLS) {
      atomic_store(&g_cmd_velocity_mps, 0.0f);
      cmd_watchdog_expired = true;
      atomic_store(&g_estop_active, hw_estop_active || cmd_watchdog_expired);
      pwm_watchdog = 0;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
  }
}

void app_main(void) {
  gpio_reset_pin(CAN_TX_IO);
  gpio_reset_pin(CAN_RX_IO);

  twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = CAN_TX_IO,
      .rx_io = CAN_RX_IO,
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 5,
      .rx_queue_len = 5,
      .alerts_enabled = TWAI_ALERT_NONE,
      .clkout_divider = 0,
  };

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(TAG_CAN, "TWAI driver started successfully");

  encoder_hw_global = init_quadrature_encoder();

  gpio_install_isr_service(0);
  gpio_isr_handler_add(ENCODER_Z_GPIO, z_isr_handler, NULL);

  pwm_init();

  g_encoder_queue = xQueueCreate(1, sizeof(encoder_msg_t));

  xTaskCreatePinnedToCore(task_can, "task_can", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(task_hardware_loop, "task_enc_pwm", 4096, NULL, 5,
                          NULL, 1);
}