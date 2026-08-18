#ifndef CUSTOM_H
#define CUSTOM_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// CAN Bus Hardware Configuration
#define CAN_TX_IO 21
#define CAN_RX_IO 22

// CAN Message IDs
#define CAN_ID_ENCODER_TX 0x123 // Transmitted: encoder position + velocity
#define CAN_ID_SPEED_CMD 0x100  // Received: speed command (32-bit float m/s)

// FORT VSC Remote (J1939 29-bit Extended CAN IDs)

#define CAN_ID_VSC_LEFT_JOYSTICK_DPAD 0x0CFDD633UL
#define CAN_ID_VSC_RIGHT_JOYSTICK_BUTTONS 0x0CFDD834UL
#define CAN_ID_VSC_HEARTBEAT 0x0CFDE801UL
#define CAN_ID_VSC_REMOTE_STATUS 0x0CFDE861UL
#define CAN_ID_VSC_USER_FB_VALUE 0x0CFDE900UL
#define CAN_ID_VSC_USER_FB_STRING 0x0CFDEA00UL

// VSC_Heartbeat E-Stop offset
#define ESTOP_BYTE_OFFSET 2

// Joystick Latch & Button Definitions

#define LATCH_SET_VALUE 1u

#define RECOVERY_LATCH_BYTE 5
#define RECOVERY_LATCH_BIT_OFFSET 6
#define RECOVERY_LATCH_MASK 0x3u

#define OPEN_LOOP_LATCH_BYTE 5
#define OPEN_LOOP_LATCH_BIT_OFFSET 0
#define OPEN_LOOP_LATCH_MASK 0x3u

// Right Joystick Y Axis
#define RIGHT_JOY_Y_STATUS_BYTE 2
#define RIGHT_JOY_Y_NEUTRAL_SHIFT 0
#define RIGHT_JOY_Y_NEGATIVE_SHIFT 2
#define RIGHT_JOY_Y_POSITIVE_SHIFT 4
#define RIGHT_JOY_Y_STATUS_MASK 0x3u
#define RIGHT_JOY_Y_MAG_LOW_BYTE 2
#define RIGHT_JOY_Y_MAG_LOW_SHIFT 6
#define RIGHT_JOY_Y_MAG_HIGH_BYTE 3
#define RIGHT_JOY_Y_MAG_MAX 1023u

// Right Joystick X Axis
#define RIGHT_JOY_X_STATUS_BYTE 0
#define RIGHT_JOY_X_NEUTRAL_SHIFT 0
#define RIGHT_JOY_X_NEGATIVE_SHIFT 2
#define RIGHT_JOY_X_POSITIVE_SHIFT 4
#define RIGHT_JOY_X_STATUS_MASK 0x3u
#define RIGHT_JOY_X_MAG_LOW_SHIFT 6
#define RIGHT_JOY_X_MAG_HIGH_BYTE 1
#define RIGHT_JOY_X_MAG_MAX 1023u

// Fort Joystick Constants
#define JOY_MAG_MAX 1023u
#define JOYSTICK_STATUS_SET 1u
#define JOYSTICK_MAX_SPEED_MPS 3.0f

// Open-Loop Accumulation Step Rate (ticks per frame at full deflection)
#define OPEN_LOOP_TICKS 50

// Fort Connection Strenght ID
#define CAN_ID_VSC_CONNECTION 0xCFDE861

// MTT DBC Telemetry CAN ID's this data come's from thebattery telemetry module
// made by MTT
#define CAN_ID_MTT_BMS_CORE_STATUS_602 1538 // 0x602: SOC

// Roboteq Articulation / Steering Controller
#define ROBOTEQ_NODE_ID 1
#define ROBOTEQ_SDO_TX_ID (0x600u + ROBOTEQ_NODE_ID)
#define ROBOTEQ_ART_CANOPEN_INDEX 0x2000u
#define ROBOTEQ_ART_CANOPEN_SUBINDEX 0x01u
#define ROBOTEQ_SDO_CS_EXPEDITED_4B 0x23u
#define ROBOTEQ_STEERING_MAX_TICKS 950

// Roboteq Emergency Stop / Release (SDO expedited write, 1 byte)
#define ROBOTEQ_ESTOP_CANOPEN_INDEX 0x200Cu
#define ROBOTEQ_RELEASE_CANOPEN_INDEX 0x200Du
#define ROBOTEQ_ESTOP_CANOPEN_SUBINDEX 0x00u
#define ROBOTEQ_SDO_CS_EXPEDITED_1B 0x2Fu

#define STEERING_SEND_DIVIDER 10
#define PWM_WATCHDOG_POLLS 30 // 30 * 10ms = 300ms timeout
#define ROBOTEQ_MAX_TICKS 950

// ---------------------------------------------------------------------------
// AMT10 Encoder Hardware
// ---------------------------------------------------------------------------
#define ENCODER_A_GPIO 26
#define ENCODER_B_GPIO 23
#define ENCODER_Z_GPIO 18
#define ENCODER_PPR 512
#define ENCODER_TICKS_PER_REV 2048

#define ENCODER_ZERO_RPM_HOLD 10
#define ENCODER_Z_DEBOUNCE_US 10000
#define ENCODER_DITHER_THRESHOLD 10
#define ENCODER_MAX_PLAUSIBLE_DELTA 20000

// ---------------------------------------------------------------------------
// PWM Output & Kelly Controller Interface
// ---------------------------------------------------------------------------
#define PWM_OUTPUT_GPIO GPIO_NUM_19
#define PWM_FREQUENCY_HZ 50000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY ((1u << 10) - 1) // 1023
#define PWM_LEDC_TIMER LEDC_TIMER_0
#define PWM_LEDC_CHANNEL LEDC_CHANNEL_0
#define PWM_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define PWM_NEUTRAL_DUTY 0

#define REVERSE_PIN GPIO_NUM_25
#define ENABLE_PIN GPIO_NUM_5

// ---------------------------------------------------------------------------
// Control Loop & PID Tuning
// ---------------------------------------------------------------------------
#define CTRL_LOOP_HZ 50
#define CTRL_LOOP_PERIOD_MS (1000 / CTRL_LOOP_HZ) // 20 ms
// 32.5 gear ratio plus 3.95m track
#define RPM_TO_MPS 0.0020153846f
#define VEL_FILTER_ALPHA 0.35f
// The plants tau was extracted from speed step responses
// The motor had no load and was identified in speed (not ideal)
// TODO : current and voltage identification

#define PLANT_TAU_S 0.32f

#define PID_KP 150.0f
#define PID_KI (PID_KP / (6.0f * PLANT_TAU_S))
#define PID_KD 20.0f

#define PID_D_FILTER_ALPHA 0.2f
#define PID_OUTPUT_FILTER_ALPHA 0.4f

#define PID_NEAR_TARGET_ENTER_FRAC 0.40f
#define PID_NEAR_TARGET_EXIT_FRAC 0.55f
#define PID_NEAR_TARGET_OUTPUT_FILTER_ALPHA 0.12f

#define PLANT_KFF_DUTY_PER_MPS 142.5f
#define TARGET_SLEW_MAX_MPS_PER_S 4.0f
#define PID_DT_S (1.0f / CTRL_LOOP_HZ)
#define FORCE_ZERO 1.0f

// ---------------------------------------------------------------------------
// Structures
// ---------------------------------------------------------------------------
typedef struct {
  float rpm;        // this cycle's (deglitched, unsmoothed) rpm
  int8_t direction; // sign of rpm: -1, 0, +1
  bool glitched;    // true if this cycle's delta was rejected as implausible
  bool z_reset;     // true if this cycle was a Z-index reset
} encoder_msg_t;

typedef struct {
  int64_t last_total;
  int64_t accumulated_count;
  int64_t last_accumulated;
  float prev_good_rpm;
} encoder_track_t;

typedef struct {
  float kp, ki, kd;
  float integral;
  float prev_measured;
  bool prev_measured_valid;
  float d_filt;
  float out_filt;
  bool out_filt_initialized;
  bool near_target_latched;
  float output_min, output_max;
  bool locked_reverse;
} pid_ctrl_t;

typedef struct {
  uint8_t status_byte;
  uint32_t neutral_status, negative_status, positive_status;
  uint32_t neutral_shift, negative_shift, positive_shift;
  uint32_t status_mask;
  uint32_t mag_shift;
  uint32_t mag_byte;
  float magnitude;
} fort_joy_state;

// ---------------------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------------------
// Hardware Drivers
pcnt_unit_handle_t init_quadrature_encoder(void);
int64_t encoder_get_total_count(pcnt_unit_handle_t unit);
void encoder_reset_count(pcnt_unit_handle_t unit);
float encoder_deglitch(encoder_track_t *track, float rpm, bool valid);
encoder_msg_t encoder_update(pcnt_unit_handle_t unit, encoder_track_t *track,
                             bool z_index_event);

void pwm_init(void);
uint32_t pwm_clamp_duty(uint32_t duty);
bool pwm_drive(pid_ctrl_t *pid, int32_t commanded, bool allow_direction_change);

// PID Control Functions
void pid_reset(pid_ctrl_t *pid);
float pid_update(pid_ctrl_t *pid, float target, float measured, float dt);

// CAN bus helper functions
void fort_joystick_read(twai_message_t msg, fort_joy_state *joy);

int can_decode(twai_message_t msg, uint32_t byte_start, uint32_t bit_offset,
               uint32_t mask);

// FORT VSC Remote Display Feedback
void vsc_send_feedback_value(uint8_t key, int32_t value);
void vsc_send_feedback_string_segment(uint8_t key, uint8_t segment,
                                      const char *str);

#endif // CUSTOM_H