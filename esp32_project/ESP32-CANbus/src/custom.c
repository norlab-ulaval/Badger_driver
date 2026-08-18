#include "custom.h"
#include "esp_log.h"
#include <stdlib.h>

#define PCNT_HIGH_LIMIT 32767
#define PCNT_LOW_LIMIT (-32768)

static portMUX_TYPE g_pcnt_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t g_pcnt_overflow = 0;
static int32_t g_pcnt_span = 0;

// ---------------------------------------------------------------------------
// Quadrature Encoder Implementation
// ---------------------------------------------------------------------------
static bool IRAM_ATTR pcnt_watchpoint_cb(pcnt_unit_handle_t unit,
                                         const pcnt_watch_event_data_t *edata,
                                         void *user_ctx) {
  portENTER_CRITICAL_ISR(&g_pcnt_mux);
  if (edata->watch_point_value == PCNT_HIGH_LIMIT) {
    g_pcnt_overflow++;
  } else if (edata->watch_point_value == PCNT_LOW_LIMIT) {
    g_pcnt_overflow--;
  }
  portEXIT_CRITICAL_ISR(&g_pcnt_mux);
  return false;
}

int64_t encoder_get_total_count(pcnt_unit_handle_t unit) {
  int raw = 0;
  int64_t overflow;

  portENTER_CRITICAL(&g_pcnt_mux);
  pcnt_unit_get_count(unit, &raw);
  overflow = g_pcnt_overflow;
  portEXIT_CRITICAL(&g_pcnt_mux);

  return overflow * (int64_t)g_pcnt_span + (int64_t)raw;
}

void encoder_reset_count(pcnt_unit_handle_t unit) {
  portENTER_CRITICAL(&g_pcnt_mux);
  pcnt_unit_clear_count(unit);
  g_pcnt_overflow = 0;
  portEXIT_CRITICAL(&g_pcnt_mux);
}

float encoder_deglitch(encoder_track_t *track, float rpm, bool valid) {
  if (valid) {
    track->prev_good_rpm = rpm;
    return rpm;
  }
  return track->prev_good_rpm;
}

encoder_msg_t encoder_update(pcnt_unit_handle_t unit, encoder_track_t *track,
                             bool z_index_event) {
  encoder_msg_t msg = {0};

  if (z_index_event) {
    encoder_reset_count(unit);
    track->last_total = 0;
    msg.z_reset = true;
  } else {
    int64_t current_total = encoder_get_total_count(unit);
    int32_t delta = (int32_t)(current_total - track->last_total);

    if (abs(delta) > ENCODER_MAX_PLAUSIBLE_DELTA) {
      track->last_total = current_total;
      msg.glitched = true;
    } else if (abs(delta) >= ENCODER_DITHER_THRESHOLD) {
      track->last_total = current_total;
      track->accumulated_count += delta;
    }
  }

  int64_t velocity =
      (track->accumulated_count - track->last_accumulated) * CTRL_LOOP_HZ;
  track->last_accumulated = track->accumulated_count;

  bool valid = !(msg.z_reset || msg.glitched);
  float raw_rpm = (float)velocity * 60.0f / (float)ENCODER_TICKS_PER_REV;
  msg.rpm = encoder_deglitch(track, raw_rpm, valid);

  msg.direction = (msg.rpm > 0.0f) ? 1 : (msg.rpm < 0.0f) ? -1 : 0;

  return msg;
}

pcnt_unit_handle_t init_quadrature_encoder(void) {
  gpio_config_t io_conf = {.pin_bit_mask = ((1ULL << ENCODER_A_GPIO) |
                                            (1ULL << ENCODER_B_GPIO) |
                                            (1ULL << ENCODER_Z_GPIO)),
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};

  gpio_config_t z_conf = {
      .pin_bit_mask = (1ULL << ENCODER_Z_GPIO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };

  gpio_config(&z_conf);
  gpio_config(&io_conf);

  pcnt_unit_config_t unit_config = {
      .high_limit = PCNT_HIGH_LIMIT,
      .low_limit = PCNT_LOW_LIMIT,
  };
  pcnt_unit_handle_t pcnt_unit = NULL;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
  g_pcnt_span = PCNT_HIGH_LIMIT - PCNT_LOW_LIMIT;

  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = ENCODER_A_GPIO,
      .level_gpio_num = ENCODER_B_GPIO,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = ENCODER_B_GPIO,
      .level_gpio_num = ENCODER_A_GPIO,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(
      pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                    PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
      pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 0));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_LOW_LIMIT));

  pcnt_event_callbacks_t pcnt_cbs = {
      .on_reach = pcnt_watchpoint_cb,
  };
  ESP_ERROR_CHECK(
      pcnt_unit_register_event_callbacks(pcnt_unit, &pcnt_cbs, NULL));

  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

  return pcnt_unit;
}

// ---------------------------------------------------------------------------
// PWM Implementation
// ---------------------------------------------------------------------------
void pwm_init(void) {
  ledc_timer_config_t timer_conf = {
      .speed_mode = PWM_LEDC_MODE,
      .timer_num = PWM_LEDC_TIMER,
      .duty_resolution = PWM_RESOLUTION,
      .freq_hz = PWM_FREQUENCY_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  ledc_channel_config_t channel_conf = {
      .speed_mode = PWM_LEDC_MODE,
      .channel = PWM_LEDC_CHANNEL,
      .timer_sel = PWM_LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = PWM_OUTPUT_GPIO,
      .duty = PWM_NEUTRAL_DUTY,
      .hpoint = 0,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

  gpio_config_t dir_conf = {
      .pin_bit_mask = (1ULL << REVERSE_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&dir_conf));
  gpio_set_level(REVERSE_PIN, 0);

  gpio_config_t enable_conf = {
      .pin_bit_mask = (1ULL << ENABLE_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&enable_conf));
  gpio_set_level(ENABLE_PIN, 1);
}

uint32_t pwm_clamp_duty(uint32_t duty) {
  return (duty > PWM_MAX_DUTY) ? PWM_MAX_DUTY : duty;
}

bool pwm_drive(pid_ctrl_t *pid, int32_t commanded,
               bool allow_direction_change) {
  bool held_by_direction_lock = false;
  uint32_t duty = 0;

  if (commanded != 0) {
    bool desired_reverse = commanded < 0;
    if (desired_reverse != pid->locked_reverse) {
      if (allow_direction_change) {
        pid->locked_reverse = desired_reverse;
      } else {
        held_by_direction_lock = true;
      }
    }
    if (!held_by_direction_lock) {
      duty = pwm_clamp_duty((uint32_t)abs(commanded));
    }
  }

  gpio_set_level(REVERSE_PIN, pid->locked_reverse ? 1 : 0);
  ledc_set_duty(PWM_LEDC_MODE, PWM_LEDC_CHANNEL, duty);
  ledc_update_duty(PWM_LEDC_MODE, PWM_LEDC_CHANNEL);

  return held_by_direction_lock;
}

// ---------------------------------------------------------------------------
// PID Control Implementation
// ---------------------------------------------------------------------------
void pid_reset(pid_ctrl_t *pid) {
  pid->integral = 0.0f;
  pid->prev_measured = 0.0f;
  pid->prev_measured_valid = false;
  pid->d_filt = 0.0f;
  pid->out_filt = 0.0f;
  pid->out_filt_initialized = false;
  pid->near_target_latched = false;
}

float pid_update(pid_ctrl_t *pid, float target, float measured, float dt) {
  float error = target - measured;
  float ff_term = target * PLANT_KFF_DUTY_PER_MPS;
  float p_term = pid->kp * error;

  float tentative_integral = pid->integral + (error * dt);
  float i_term = pid->ki * tentative_integral;

  float d_raw = 0.0f;
  if (dt > 0.0f && pid->prev_measured_valid) {
    d_raw = -pid->kd * ((measured - pid->prev_measured) / dt);
  }
  pid->prev_measured = measured;
  pid->prev_measured_valid = true;
  pid->d_filt += PID_D_FILTER_ALPHA * (d_raw - pid->d_filt);
  float d_term = pid->d_filt;

  float raw_out = ff_term + p_term + i_term + d_term;

  if (target != 0.0f) {
    float abs_frac = fabsf(error) / fabsf(target);
    if (!pid->near_target_latched && abs_frac <= PID_NEAR_TARGET_ENTER_FRAC) {
      pid->near_target_latched = true;
    } else if (pid->near_target_latched &&
               abs_frac > PID_NEAR_TARGET_EXIT_FRAC) {
      pid->near_target_latched = false;
    }
  } else {
    pid->near_target_latched = false;
  }
  float output_alpha = pid->near_target_latched
                           ? PID_NEAR_TARGET_OUTPUT_FILTER_ALPHA
                           : PID_OUTPUT_FILTER_ALPHA;

  if (!pid->out_filt_initialized) {
    pid->out_filt = raw_out;
    pid->out_filt_initialized = true;
  } else {
    pid->out_filt += output_alpha * (raw_out - pid->out_filt);
  }

  float output = pid->out_filt;
  if (output > pid->output_max) {
    output = pid->output_max;
  } else if (output < pid->output_min) {
    output = pid->output_min;
  }

  bool saturated = (output != pid->out_filt);
  bool pushing_further = (pid->out_filt > 0.0f && error > 0.0f) ||
                         (pid->out_filt < 0.0f && error < 0.0f);

  if (!saturated || !pushing_further) {
    pid->integral = tentative_integral;
  }

  return output;
}

void fort_joystick_read(twai_message_t msg, fort_joy_state *joy) {
  uint32_t current_status = msg.data[joy->status_byte];

  joy->neutral_status =
      can_decode(msg, joy->status_byte, joy->neutral_shift, joy->status_mask);
  joy->negative_status =
      can_decode(msg, joy->status_byte, joy->negative_shift, joy->status_mask);
  joy->positive_status =
      can_decode(msg, joy->status_byte, joy->positive_shift, joy->status_mask);

  joy->magnitude = (float)(((current_status >> joy->mag_shift) & 0x3u) |
                           ((uint32_t)msg.data[joy->mag_byte] << 2)) /
                   JOY_MAG_MAX;

  if (joy->magnitude > 1.0f)
    joy->magnitude = 1.0f;
}

int can_decode(twai_message_t msg, uint32_t byte_start, uint32_t bit_offset,
               uint32_t mask) {

  return (msg.data[byte_start] >> bit_offset & mask);
}

// ---------------------------------------------------------------------------
// FORT Display Telemetry Helpers
// ---------------------------------------------------------------------------
void vsc_send_feedback_value(uint8_t key, int32_t value) {
  twai_message_t msg = {
      .identifier = CAN_ID_VSC_USER_FB_VALUE,
      .data_length_code = 5,
      .flags = TWAI_MSG_FLAG_EXTD,
  };

  msg.data[0] = key;
  memcpy(&msg.data[1], &value, sizeof(int32_t));

  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void vsc_send_feedback_string_segment(uint8_t key, uint8_t segment,
                                      const char *str) {
  twai_message_t msg = {
      .identifier = CAN_ID_VSC_USER_FB_STRING,
      .data_length_code = 8,
      .flags = TWAI_MSG_FLAG_EXTD,
  };

  msg.data[0] = key;
  msg.data[1] = segment;

  memset(&msg.data[2], 0, 6);

  if (str != NULL) {
    size_t len = strlen(str);
    if (len > 6)
      len = 6;
    memcpy(&msg.data[2], str, len);
  }

  twai_transmit(&msg, pdMS_TO_TICKS(10));
}