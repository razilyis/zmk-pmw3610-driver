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
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
  int64_t last_smp_time;
  int64_t last_rpt_time;
#endif

  struct gpio_callback irq_gpio_cb; // motion pin irq callback
  struct k_work trigger_work;       // realtrigger job
  struct k_work_delayable inertia_work; // delayed inertial scroll job
  struct k_mutex spi_mutex;         // serialize multi-transfer sensor commands

  struct k_work_delayable
      init_work; // the work structure for delayable init steps
  int async_init_step;
  uint8_t init_retries; // counter for async init retries
  uint8_t report_error_count;
  int32_t inertia_vx_q8;
  int32_t inertia_vy_q8;
  int32_t inertia_rx_q8;
  int32_t inertia_ry_q8;
  int32_t gesture_vx_q8;
  int32_t gesture_vy_q8;
  int64_t gesture_last_motion_ms;
  int16_t micro_x_pending;
  int16_t micro_y_pending;
  int64_t micro_x_last_motion_ms;
  int64_t micro_y_last_motion_ms;
  int8_t micro_x_direction;
  int8_t micro_y_direction;
  bool inertial_scroll_enabled;
  bool vertical_scroll_inverted;
  bool horizontal_scroll_inverted;

  bool ready; // whether init is finished successfully
  int err;    // error code during async init
};

// device config data structure
struct pixart_config {
  struct spi_dt_spec spi;
  struct gpio_dt_spec irq_gpio;
  uint16_t cpi;
  uint16_t motion_threshold;
  bool low_speed_stabilizer;
  uint16_t low_speed_stabilizer_threshold;
  uint16_t low_speed_stabilizer_timeout_ms;
  bool swap_xy;
  bool inv_x;
  bool inv_y;
  uint8_t evt_type;
  uint8_t x_input_code;
  uint8_t y_input_code;
  bool force_awake;
  bool force_awake_4ms_mode;
  bool inertial_scroll;
  uint16_t inertial_scroll_decay_basis_points;
  uint16_t inertial_scroll_interval_ms;
  uint16_t inertial_scroll_threshold;
  uint16_t inertial_scroll_gain_pct;
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
