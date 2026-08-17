#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct pixart_data {
  const struct device *dev;
  bool sw_smart_flag; // for pmw3610 smart algorithm

  int64_t dx;
  int64_t dy;
  int64_t last_smp_time;
  int64_t last_rpt_time;

  struct gpio_callback irq_gpio_cb; // motion pin irq callback
  struct k_work_delayable trigger_work;
  struct k_work_delayable performance_work;
  struct k_work_delayable init_work;
  struct k_work_delayable inertia_work;
  struct k_mutex inertia_mutex;

  int async_init_step;
  uint8_t init_retries;
  uint8_t report_error_count;
  uint8_t no_motion_irq_count;
  int64_t no_motion_irq_since_ms;
  int64_t input_retry_since_ms;
  atomic_t performance_requested;

  int32_t inertia_x;
  int32_t inertia_y;
  int32_t inertia_accum_x;
  int32_t inertia_accum_y;
  int64_t inertia_start_time;
  int32_t gesture_vx;
  int32_t gesture_vy;
  int64_t last_motion_time;

  bool input_retry_pending;
  bool input_frame_open;
  bool irq_recheck_pending;

  bool ready;
  int err;
};

// device config data structure
struct pixart_config {
  struct spi_dt_spec spi;
  struct gpio_dt_spec irq_gpio;
  uint16_t cpi;
  uint16_t motion_threshold;
  uint16_t max_motion_delta;
  uint16_t max_report_delta;
  bool swap_xy;
  bool inv_x;
  bool inv_y;
  uint8_t evt_type;
  uint16_t x_input_code;
  uint16_t y_input_code;
  bool force_awake;
  bool force_awake_4ms_mode;
  bool inertial_scroll;
  bool scroll_direction_toggle;
  uint16_t inertial_scroll_decay_basis_points;
  uint16_t inertial_scroll_interval_ms;
  uint16_t inertial_scroll_threshold;
  uint16_t inertial_scroll_gain_pct;
  uint16_t inertial_scroll_max_velocity;
  uint16_t inertial_scroll_max_duration_ms;
  uint16_t inertial_scroll_fade_duration_ms;
  bool vertical_scroll_uses_x_axis;
  const uint8_t *inertial_scroll_layers;
  size_t inertial_scroll_layer_count;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
