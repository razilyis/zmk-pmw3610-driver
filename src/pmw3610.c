/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include "pmw3610.h"
#include "pmw3610_control.h"
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/keymap.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
  ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
  ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
  ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after
                             // self-test check
  ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time
                             // (run, rest1, rest2) and clear motion registers

  ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes
 * succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do
// other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] =
        10 + CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS, // >10ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] =
        200, // 150 us required, test shows too short,
             // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50, // 10 ms required in spec,
                                      // test shows too short,
                                      // especially when integrated with
                                      // display, > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);
static void pmw3610_inertia_work_callback(struct k_work *work);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(
    const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_read_reg(const struct device *dev, uint8_t addr,
                            uint8_t *value);
static int pmw3610_write_reg(const struct device *dev, uint8_t addr,
                             uint8_t value);
static int pmw3610_set_interrupt(const struct device *dev, bool en);

static bool pmw3610_supports_inertia(const struct device *dev) {
  const struct pixart_config *config = dev->config;

  return config->inertial_scroll;
}

static bool pmw3610_supports_scroll_direction(const struct device *dev) {
  const struct pixart_config *config = dev->config;

  return config->inertial_scroll || config->scroll_direction_toggle;
}

static int32_t pmw3610_abs32(int32_t value) {
  if (value == INT32_MIN) {
    return INT32_MAX;
  }
  return value < 0 ? -value : value;
}

static bool pmw3610_inertial_layer_active(const struct pixart_config *config) {
  if (config->inertial_scroll_layer_count == 0) {
    return true;
  }

  for (size_t i = 0; i < config->inertial_scroll_layer_count; i++) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT) &&                                           \
    !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    if (pmw3610_control_remote_layer_active(
            config->inertial_scroll_layers[i])) {
#else
    if (zmk_keymap_layer_active(config->inertial_scroll_layers[i])) {
#endif
      return true;
    }
  }

  return false;
}

static void pmw3610_clear_inertia_locked(struct pixart_data *data) {
  data->inertia_generation++;
  data->inertia_vx_q8 = 0;
  data->inertia_vy_q8 = 0;
  data->inertia_rx_q8 = 0;
  data->inertia_ry_q8 = 0;
  data->inertia_started_ms = 0;
}

static void pmw3610_clear_gesture_velocity_locked(struct pixart_data *data) {
  data->gesture_vx_q8 = 0;
  data->gesture_vy_q8 = 0;
}

static void pmw3610_stop_inertia(struct pixart_data *data) {
  /*
   * Cancel first, then take the mutex. If the callback is already running,
   * waiting for the mutex guarantees that its last update is cleared here.
   */
  k_work_cancel_delayable(&data->inertia_work);
  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  pmw3610_clear_inertia_locked(data);
  k_mutex_unlock(&data->inertia_mutex);
}

static void pmw3610_reset_gesture_velocity(struct pixart_data *data) {
  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  pmw3610_clear_gesture_velocity_locked(data);
  k_mutex_unlock(&data->inertia_mutex);
}

static void pmw3610_reset_micro_motion(struct pixart_data *data) {
  data->micro_x_pending = 0;
  data->micro_y_pending = 0;
  data->micro_x_last_motion_ms = 0;
  data->micro_y_last_motion_ms = 0;
  data->micro_x_direction = 0;
  data->micro_y_direction = 0;
}

static int16_t pmw3610_stabilize_micro_axis(
    int16_t value, int16_t *pending, int8_t *direction,
    int64_t *last_motion_ms, const struct pixart_config *config, int64_t now) {
  bool direction_history_expired =
      *last_motion_ms > 0 &&
      now - *last_motion_ms > config->low_speed_stabilizer_timeout_ms;
  if (direction_history_expired) {
    *pending = 0;
    *direction = 0;
  }

  if (value == 0) {
    return 0;
  }

  *last_motion_ms = now;
  int8_t value_direction = value > 0 ? 1 : -1;

  /*
   * Do not delay the first micro movement after startup or an idle pause.
   * Confirmation is only needed for a genuine reversal of an established
   * direction, where it filters one-sample sensor bounce.
   */
  if (*direction == 0) {
    *pending = 0;
    *direction = value_direction;
    return value;
  }

  if (pmw3610_abs32(value) > config->low_speed_stabilizer_threshold) {
    *pending = 0;
    *direction = value_direction;
    return value;
  }

  if (*direction == value_direction) {
    *pending = 0;
    return value;
  }

  if (*pending != 0 &&
      ((*pending > 0 && value < 0) || (*pending < 0 && value > 0))) {
    *pending += value;
    return 0;
  }

  *pending += value;
  if (pmw3610_abs32(*pending) <=
      config->low_speed_stabilizer_threshold) {
    return 0;
  }

  int16_t stabilized = *pending;
  *pending = 0;
  *direction = stabilized > 0 ? 1 : -1;
  return stabilized;
}

static void pmw3610_apply_low_speed_stabilizer(const struct device *dev,
                                                int16_t *x, int16_t *y) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;

  bool scroll_layer_active =
      pmw3610_supports_inertia(dev) &&
      pmw3610_inertial_layer_active(config);
  if (!config->low_speed_stabilizer || scroll_layer_active) {
    pmw3610_reset_micro_motion(data);
    return;
  }

  int64_t now = k_uptime_get();
  *x = pmw3610_stabilize_micro_axis(
      *x, &data->micro_x_pending, &data->micro_x_direction,
      &data->micro_x_last_motion_ms, config, now);
  *y = pmw3610_stabilize_micro_axis(
      *y, &data->micro_y_pending, &data->micro_y_direction,
      &data->micro_y_last_motion_ms, config, now);
}

static void pmw3610_begin_recovery(struct pixart_data *data) {
  const struct device *dev = data->dev;

  if (!data->ready) {
    return;
  }

  data->ready = false;
  pmw3610_set_interrupt(dev, false);
  data->dx = 0;
  data->dy = 0;
  data->report_error_count = 0;
  data->no_motion_irq_count = 0;
  pmw3610_stop_inertia(data);
  pmw3610_reset_gesture_velocity(data);
  pmw3610_reset_micro_motion(data);
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
  data->last_smp_time = 0;
  data->last_rpt_time = 0;
#endif
  data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
  data->init_retries = 0;
  k_work_reschedule(&data->init_work, K_NO_WAIT);
}

static int32_t pmw3610_filter_gesture_velocity(int32_t previous_q8,
                                               int32_t sample_q8) {
  if (previous_q8 == 0) {
    return sample_q8;
  }

  if ((previous_q8 < 0) != (sample_q8 < 0) && sample_q8 != 0) {
    return sample_q8;
  }

  /*
   * Track acceleration quickly, but release a captured flick velocity slowly.
   * This avoids basing momentum solely on the final, naturally slower sample
   * while keeping gentle movements gentle.
   */
  int32_t previous_pct =
      pmw3610_abs32(sample_q8) >= pmw3610_abs32(previous_q8)
          ? PMW3610_INERTIA_ATTACK_PREVIOUS_PCT
          : PMW3610_INERTIA_RELEASE_PREVIOUS_PCT;

  return (int32_t)(((int64_t)previous_q8 * previous_pct +
                    (int64_t)sample_q8 * (100 - previous_pct)) /
                   100);
}

static int32_t pmw3610_normalize_motion_q8(
    int16_t sample, int64_t sample_interval_ms,
    const struct pixart_config *config) {
  int64_t max_sample_interval_ms =
      MAX(PMW3610_INERTIA_GESTURE_TIMEOUT_MS,
          MAX(CONFIG_PMW3610_REST1_SAMPLE_TIME_MS,
              MAX(CONFIG_PMW3610_REST2_SAMPLE_TIME_MS,
                  CONFIG_PMW3610_REST3_SAMPLE_TIME_MS)));
  sample_interval_ms = CLAMP(sample_interval_ms, 1, max_sample_interval_ms);
  int64_t normalized =
      (int64_t)sample * PMW3610_INERTIA_SCALE *
      config->inertial_scroll_interval_ms / sample_interval_ms;

  return (int32_t)CLAMP(normalized, INT32_MIN, INT32_MAX);
}

static int64_t pmw3610_first_gesture_sample_interval_ms(
    const struct pixart_config *config, int64_t now,
    int64_t time_since_last_motion_ms, bool performance_mode_enabled,
    int64_t performance_mode_disabled_ms) {
  int64_t run_interval_ms = config->force_awake_4ms_mode ? 4 : 8;
  int64_t report_interval_ms =
      MAX(run_interval_ms, CONFIG_PMW3610_REPORT_INTERVAL_MIN);

  if (performance_mode_enabled) {
    return report_interval_ms;
  }

  int64_t low_power_elapsed_ms = time_since_last_motion_ms;
  if (config->force_awake && performance_mode_disabled_ms > 0) {
    low_power_elapsed_ms = now - performance_mode_disabled_ms;
  }

  int64_t sample_interval_ms;
  if (low_power_elapsed_ms < CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS) {
    sample_interval_ms = run_interval_ms;
  } else if (low_power_elapsed_ms <
             (int64_t)CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS +
                 (int64_t)CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS) {
    sample_interval_ms = CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
  } else if (low_power_elapsed_ms <
             (int64_t)CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS +
                 (int64_t)CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS +
                 (int64_t)CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS) {
    sample_interval_ms = CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
  } else {
    sample_interval_ms = CONFIG_PMW3610_REST3_SAMPLE_TIME_MS;
  }

  return MAX(sample_interval_ms, report_interval_ms);
}

static int pmw3610_emit_input(const struct device *dev, int16_t x, int16_t y) {
  const struct pixart_config *config = dev->config;
  bool have_x = x != 0;
  bool have_y = y != 0;
  k_timeout_t timeout = K_MSEC(CONFIG_PMW3610_INPUT_REPORT_TIMEOUT_MS);
  int err;

  if (!have_x && !have_y) {
    return 0;
  }

  /*
   * Keep a two-axis sample in one synchronized frame for smooth diagonal
   * motion. Every queue wait is finite so a stalled input consumer cannot
   * block the system work queue indefinitely.
   */
  if (have_x) {
    err = input_report(dev, config->evt_type, config->x_input_code, x, !have_y,
                       timeout);
    if (err) {
      return err;
    }
  }
  if (have_y) {
    err = input_report(dev, config->evt_type, config->y_input_code, y, true,
                       timeout);
    if (err) {
      if (have_x) {
        /*
         * X was queued without sync. Close that partial frame with a neutral
         * event when possible so it cannot be merged into unrelated motion.
         * This recovery attempt is also bounded.
         */
        int flush_err =
            input_report(dev, config->evt_type, config->x_input_code, 0, true,
                         timeout);
        if (flush_err) {
          LOG_WRN("Failed to close partial PMW3610 input frame: %d",
                  flush_err);
        }
      }
      return err;
    }
  }

  return 0;
}

static int16_t pmw3610_q8_to_step(int32_t *remainder_q8,
                                  int32_t velocity_q8) {
  int32_t total_q8 = *remainder_q8 + velocity_q8;
  int32_t step = total_q8 / PMW3610_INERTIA_SCALE;

  *remainder_q8 = total_q8 - (step * PMW3610_INERTIA_SCALE);
  return (int16_t)CLAMP(step, INT16_MIN, INT16_MAX);
}

static bool pmw3610_inertia_active(const struct pixart_data *data,
                                   const struct pixart_config *config) {
  return pmw3610_abs32(data->inertia_vx_q8) >=
             config->inertial_scroll_threshold ||
         pmw3610_abs32(data->inertia_vy_q8) >=
             config->inertial_scroll_threshold;
}

static void pmw3610_schedule_inertia(struct pixart_data *data,
                                     const struct pixart_config *config) {
  k_work_reschedule(&data->inertia_work,
                    K_MSEC(config->inertial_scroll_interval_ms));
}

static void pmw3610_update_inertia_from_motion(
    struct pixart_data *data, const struct pixart_config *config, int16_t rx,
    int16_t ry, int64_t motion_time_ms, bool performance_mode_enabled,
    int64_t performance_mode_disabled_ms) {
  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  int64_t now = motion_time_ms;
  int64_t sample_interval_ms = config->inertial_scroll_interval_ms;
  if (data->gesture_last_motion_ms != 0) {
    sample_interval_ms = now - data->gesture_last_motion_ms;
  }
  bool new_gesture =
      data->gesture_last_motion_ms == 0 ||
      sample_interval_ms > PMW3610_INERTIA_GESTURE_TIMEOUT_MS;
  if (new_gesture) {
    pmw3610_clear_gesture_velocity_locked(data);
    sample_interval_ms = pmw3610_first_gesture_sample_interval_ms(
        config, now, sample_interval_ms, performance_mode_enabled,
        performance_mode_disabled_ms);
  }

  data->gesture_vx_q8 = pmw3610_filter_gesture_velocity(
      data->gesture_vx_q8,
      pmw3610_normalize_motion_q8(rx, sample_interval_ms, config));
  data->gesture_vy_q8 = pmw3610_filter_gesture_velocity(
      data->gesture_vy_q8,
      pmw3610_normalize_motion_q8(ry, sample_interval_ms, config));
  data->gesture_last_motion_ms = now;

  int32_t max_velocity_q8 =
      (int32_t)config->inertial_scroll_max_velocity * PMW3610_INERTIA_SCALE;
  int64_t scaled_vx_q8 =
      (int64_t)data->gesture_vx_q8 * config->inertial_scroll_gain_pct / 100;
  int64_t scaled_vy_q8 =
      (int64_t)data->gesture_vy_q8 * config->inertial_scroll_gain_pct / 100;
  data->inertia_vx_q8 =
      (int32_t)CLAMP(scaled_vx_q8, -(int64_t)max_velocity_q8,
                     (int64_t)max_velocity_q8);
  data->inertia_vy_q8 =
      (int32_t)CLAMP(scaled_vy_q8, -(int64_t)max_velocity_q8,
                     (int64_t)max_velocity_q8);

  if (pmw3610_inertia_active(data, config)) {
    data->inertia_started_ms = now;
    pmw3610_schedule_inertia(data, config);
  }
  k_mutex_unlock(&data->inertia_mutex);
}

bool pmw3610_inertial_scroll_is_enabled(const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;

  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  bool enabled =
      pmw3610_supports_inertia(dev) && data->inertial_scroll_enabled;
  k_mutex_unlock(&data->inertia_mutex);
  return enabled && pmw3610_inertial_layer_active(config);
}

int pmw3610_set_inertial_scroll_enabled(const struct device *dev,
                                        bool enabled) {
  struct pixart_data *data = dev->data;

  if (!pmw3610_supports_inertia(dev) && enabled) {
    return -ENOTSUP;
  }

  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  data->inertial_scroll_enabled =
      enabled && pmw3610_supports_inertia(dev);
  if (!data->inertial_scroll_enabled) {
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
  }
  k_mutex_unlock(&data->inertia_mutex);
  if (!enabled) {
    k_work_cancel_delayable(&data->inertia_work);
  }

  return 0;
}

bool pmw3610_vertical_scroll_direction_is_inverted(const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;

  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  bool inverted = data->vertical_scroll_inverted;
  k_mutex_unlock(&data->inertia_mutex);
  return pmw3610_supports_scroll_direction(dev) && inverted &&
         pmw3610_inertial_layer_active(config);
}

bool pmw3610_horizontal_scroll_direction_is_inverted(
    const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;

  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  bool inverted = data->horizontal_scroll_inverted;
  k_mutex_unlock(&data->inertia_mutex);
  return pmw3610_supports_scroll_direction(dev) && inverted &&
         pmw3610_inertial_layer_active(config);
}

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value,
                        uint8_t len) {
  const struct pixart_config *cfg = dev->config;
  struct pixart_data *data = dev->data;
  struct spi_config read_config = cfg->spi.config;
  const struct spi_buf tx_buf = {.buf = &addr, .len = sizeof(addr)};
  const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
  struct spi_buf rx_buf = {.buf = value, .len = len};
  const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

  /*
   * PMW3610 requires CS to stay asserted while waiting tSRAD between the
   * address byte and the first data byte. A single transceive keeps CS low but
   * provides no inter-byte delay, so split the operation while locking the SPI
   * bus and holding CS. spi_release() below forcibly releases both on every
   * exit path.
   */
  read_config.operation |= SPI_HOLD_ON_CS | SPI_LOCK_ON;
  k_mutex_lock(&data->spi_mutex, K_FOREVER);

  int err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                              PMW3610_SPI_CLOCK_CMD_ENABLE);
  if (err) {
    k_mutex_unlock(&data->spi_mutex);
    return err;
  }
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  err = spi_write(cfg->spi.bus, &read_config, &tx);
  if (!err) {
    k_busy_wait(T_SRAD_DELAY_US);
    err = spi_read(cfg->spi.bus, &read_config, &rx);
  }

  int release_err = spi_release(cfg->spi.bus, &read_config);
  int disable_err =
      pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                        PMW3610_SPI_CLOCK_CMD_DISABLE);
  k_mutex_unlock(&data->spi_mutex);

  if (err) {
    return err;
  }
  if (release_err) {
    return release_err;
  }
  return disable_err;
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr,
                            uint8_t *value) {
  return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr,
                             uint8_t value) {
  const struct pixart_config *cfg = dev->config;
  uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
  const struct spi_buf tx_buf = {
      .buf = write_buf,
      .len = sizeof(write_buf),
  };
  const struct spi_buf_set tx = {
      .buffers = &tx_buf,
      .count = 1,
  };
  return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
  struct pixart_data *data = dev->data;
  k_mutex_lock(&data->spi_mutex, K_FOREVER);

  int err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                              PMW3610_SPI_CLOCK_CMD_ENABLE);
  if (err) {
    k_mutex_unlock(&data->spi_mutex);
    return err;
  }
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  err = pmw3610_write_reg(dev, reg, val);
  int disable_err =
      pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                        PMW3610_SPI_CLOCK_CMD_DISABLE);
  k_mutex_unlock(&data->spi_mutex);

  return err ? err : disable_err;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi, bool swap_xy,
                           bool inv_x, bool inv_y) {
  struct pixart_data *driver_data = dev->data;
  /* Set resolution with CPI step of 200 cpi
   * 0x1: 200 cpi (minimum cpi)
   * 0x2: 400 cpi
   * 0x3: 600 cpi
   * :
   */

  if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
    LOG_ERR("CPI value %u out of range", cpi);
    return -EINVAL;
  }

  uint8_t value = 0x00;
  int err = 0;

  LOG_INF("Setting cpi: %d", cpi);
  // Convert CPI to register value
  // Set prefered RES_STEP
  //   BIT 4-0: CPI
  uint8_t cpi_val = cpi / 200;
  value = (value & 0xE0) | (cpi_val & 0x1F);

  // Convert axis to register value
  // Set prefered RES_STEP
  //   BIT 7: SWAP_XY
  //   BIT 6: INV_X
  //   BIT 5: INV_Y
  LOG_INF("Setting axis swap_xy: %s inv_x: %s inv_y: %s",
          swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

#if IS_ENABLED(CONFIG_PMW3610_SWAP_XY)
  value |= (1 << 7);
#else
  if (swap_xy) {
    value |= (1 << 7);
  } else {
    value &= ~(1 << 7);
  }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_X)
  value |= (1 << 6);
#else
  if (inv_x) {
    value |= (1 << 6);
  } else {
    value &= ~(1 << 6);
  }
#endif
#if IS_ENABLED(CONFIG_PMW3610_INVERT_Y)
  value |= (1 << 5);
#else
  if (inv_y) {
    value |= (1 << 5);
  } else {
    value &= ~(1 << 5);
  }
#endif

  LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

  /* set the cpi */
  uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
  uint8_t data[] = {0xFF, value, 0x00};

  k_mutex_lock(&driver_data->spi_mutex, K_FOREVER);
  err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                          PMW3610_SPI_CLOCK_CMD_ENABLE);
  if (err) {
    k_mutex_unlock(&driver_data->spi_mutex);
    return err;
  }
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  /* Write data */
  for (size_t i = 0; i < sizeof(data); i++) {
    err = pmw3610_write_reg(dev, addr[i], data[i]);
    if (err) {
      LOG_ERR("Burst write failed on SPI write (data)");
      break;
    }
  }
  int disable_err =
      pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                        PMW3610_SPI_CLOCK_CMD_DISABLE);
  if (!err) {
    err = disable_err;
  }
  k_mutex_unlock(&driver_data->spi_mutex);

  if (err) {
    LOG_ERR("Failed to set CPI");
    return err;
  }

  return 0;
}

/* Set sampling rate in each mode (in ms) */
static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr,
                                   uint32_t sample_time) {
  uint32_t maxtime = 2550;
  uint32_t mintime = 10;
  if ((sample_time > maxtime) || (sample_time < mintime)) {
    LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime,
            maxtime);
    return -EINVAL;
  }

  uint8_t value = sample_time / mintime;
  LOG_INF("Set sample time to %u ms (reg value: 0x%x)", sample_time, value);

  /* The sample time is (reg_value * mintime ) ms. 0x00 is rounded to 0x1 */
  int err = pmw3610_write(dev, reg_addr, value);
  if (err) {
    LOG_ERR("Failed to change sample time");
  }

  return err;
}

/* Set downshift time in ms. */
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is
// hard coded to be 4 ms The pos-mode rate is configured in
// pmw3610_async_init_configure
static int pmw3610_set_downshift_time(const struct device *dev,
                                      uint8_t reg_addr, uint32_t time) {
  uint32_t maxtime;
  uint32_t mintime;

  switch (reg_addr) {
  case PMW3610_REG_RUN_DOWNSHIFT:
    /*
     * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
     *                      * 8 * pos-rate (fixed to 4ms)
     */
    maxtime = 8160; // 32 * 255;
    mintime = 32;   // hard-coded in pmw3610_async_init_configure
    break;

  case PMW3610_REG_REST1_DOWNSHIFT:
    /*
     * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
     *                        * 16 * Rest1_sample_period (default 40 ms)
     */
    maxtime = 255 * 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
    mintime = 16 * CONFIG_PMW3610_REST1_SAMPLE_TIME_MS;
    break;

  case PMW3610_REG_REST2_DOWNSHIFT:
    /*
     * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
     *                        * 128 * Rest2 rate (default 100 ms)
     */
    maxtime = 255 * 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
    mintime = 128 * CONFIG_PMW3610_REST2_SAMPLE_TIME_MS;
    break;

  default:
    LOG_ERR("Not supported");
    return -ENOTSUP;
  }

  if ((time > maxtime) || (time < mintime)) {
    LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
    return -EINVAL;
  }

  __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

  /* Convert time to register value */
  uint8_t value = time / mintime;

  LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

  int err = pmw3610_write(dev, reg_addr, value);
  if (err) {
    LOG_ERR("Failed to change downshift time");
  }

  return err;
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
  const struct pixart_config *config = dev->config;
  struct pixart_data *data = dev->data;
  int err = 0;

  if (config->force_awake) {
    uint8_t value;
    err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
    if (err) {
      LOG_ERR("Can't read ref-performance %d", err);
      return err;
    }
    LOG_INF("Get performance register (reg value 0x%x)", value);

    // Set prefered RUN RATE
    //   BIT 3:   VEL_RUNRATE    0x0: 8ms; 0x1 4ms;
    //   BIT 2:   POSHI_RUN_RATE 0x0: 8ms; 0x1 4ms;
    //   BIT 1-0: POSLO_RUN_RATE 0x0: 8ms; 0x1 4ms; 0x2 2ms; 0x4 Reserved
    // Note: The 'value' read here is used only for the `if (perf != value)`
    // optimization check below. When force_awake is disabled, we need to clear
    // both the run rate (lower nibble) and awake flag (upper nibble).
    uint8_t perf = 0x00; // Default to IDLE state (8ms tick, no force awake)

    if (enabled) {
      if (config->force_awake_4ms_mode) {
        perf = 0x0d; // RUN RATE @ 4ms
      }
      perf |= 0xF0; // set bit[7..4] to 0xF (force awake)
    }
    if (perf != value) {
      err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
      if (err) {
        LOG_ERR("Can't write performance register %d", err);
        return err;
      }
      LOG_INF("Set performance register (reg value 0x%x)", perf);
    }
    LOG_INF("%s performance mode", enabled ? "enable" : "disable");
  }

  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  data->performance_mode_enabled = config->force_awake && enabled;
  data->performance_mode_disabled_ms =
      data->performance_mode_enabled ? 0 : k_uptime_get();
  k_mutex_unlock(&data->inertia_mutex);

  return err;
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
  const struct pixart_config *config = dev->config;
  // Revert to level trigger to avoid missing edges during interrupt disable
  int ret = gpio_pin_interrupt_configure_dt(
      &config->irq_gpio, en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
  if (ret < 0) {
    LOG_ERR("can't set interrupt");
  }
  return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
  int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET,
                              PMW3610_POWERUP_CMD_RESET);
  if (ret < 0) {
    return ret;
  }
  return 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
  return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
  uint8_t value;
  int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
  if (err) {
    LOG_ERR("Can't do self-test");
    return err;
  }

  if ((value & 0x0F) != 0x0F) {
    LOG_ERR("Failed self-test (0x%x)", value);
    return -EINVAL;
  }

  uint8_t product_id = 0x01;
  err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
  if (err) {
    LOG_ERR("Cannot obtain product id");
    return err;
  }

  if (product_id != PMW3610_PRODUCT_ID) {
    LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id,
            PMW3610_PRODUCT_ID);
    return -EIO;
  }

  return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
  int err = 0;
  const struct pixart_config *config = dev->config;

  // clear motion registers first (required in datasheet)
  for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
    uint8_t buf[1];
    err = pmw3610_read_reg(dev, reg, buf);
  }

  if (!err) {
    err = pmw3610_set_performance(dev, true);
  }

  if (!err) {
    err = pmw3610_set_cpi(dev, config->cpi, config->swap_xy, config->inv_x,
                          config->inv_y);
  }

  if (!err) {
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                     CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS);
  }

  if (!err) {
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                     CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS);
  }

  if (!err) {
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                     CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS);
  }

  if (!err) {
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                  CONFIG_PMW3610_REST1_SAMPLE_TIME_MS);
  }

  if (!err) {
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                  CONFIG_PMW3610_REST2_SAMPLE_TIME_MS);
  }

  if (!err) {
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                  CONFIG_PMW3610_REST3_SAMPLE_TIME_MS);
  }

  if (err) {
    LOG_ERR("Config the sensor failed");
    return err;
  }

  return 0;
}

static void pmw3610_async_init(struct k_work *work) {
  struct k_work_delayable *work2 = (struct k_work_delayable *)work;
  struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
  const struct device *dev = data->dev;

  LOG_INF("PMW3610 async init step %d", data->async_init_step);

  data->err = async_init_fn[data->async_init_step](dev);
  if (data->err) {
    if (data->init_retries < 3) {
      data->init_retries++;
      LOG_ERR("PMW3610 initialization failed in step %d, retrying (%d/3)...",
              data->async_init_step, data->init_retries);
      // Retry the current step after a short delay (100ms) to provide a
      // fail-safe against temporary SPI/sensor issues
      k_work_schedule(&data->init_work, K_MSEC(100));
    } else {
      LOG_ERR("PMW3610 initialization failed in step %d after 3 retries. "
              "Retrying the full sequence in %d ms...",
              data->async_init_step, CONFIG_PMW3610_INIT_RETRY_BACKOFF_MS);
      /*
       * Prevent a tight SPI retry loop without making a temporary sensor
       * failure look like a ten-second freeze. This remains asynchronous, so
       * the keyboard and the other split half continue to work.
       */
      data->init_retries = 0;
      data->async_init_step =
          ASYNC_INIT_STEP_POWER_UP; // Restart the entire initialization flow
                                    // from the beginning
      k_work_schedule(
          &data->init_work,
          K_MSEC(CONFIG_PMW3610_INIT_RETRY_BACKOFF_MS));
    }
  } else {

    data->init_retries = 0; // Reset retries on success
    data->async_init_step++;

    if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
      /*
       * Anchor the first motion sample to a real elapsed interval too. This
       * prevents a REST sample accumulated after startup from being treated
       * as one inertia tick.
       */
      data->gesture_last_motion_ms = k_uptime_get();
      data->ready = true; // sensor is ready to work
      LOG_INF("PMW3610 initialized");
      data->err = pmw3610_set_interrupt(dev, true);
      if (data->err) {
        LOG_ERR("Failed to enable PMW3610 interrupt; restarting init");
        data->ready = false;
        data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
        k_work_reschedule(&data->init_work, K_MSEC(100));
      }
    } else {
      k_work_schedule(&data->init_work,
                      K_MSEC(async_init_delay[data->async_init_step]));
    }
  }
}

static int pmw3610_report_data(const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;
  uint8_t buf[PMW3610_BURST_SIZE];

  if (unlikely(!data->ready)) {
    LOG_WRN("Device is not initialized yet");
    return -EBUSY;
  }

  int64_t now = k_uptime_get();

  int err =
      pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
  if (err) {
    data->report_error_count++;
    LOG_WRN("Motion read failed (%d/%d): %d", data->report_error_count,
            PMW3610_REPORT_ERROR_RECOVERY_COUNT, err);
    if (data->report_error_count >= PMW3610_REPORT_ERROR_RECOVERY_COUNT) {
      LOG_ERR("Repeated motion read failures; restarting PMW3610");
      pmw3610_begin_recovery(data);
    }
    return err;
  }
  // Check FAULT bit
  if (unlikely(buf[0] & PMW3610_MOTION_FAULT)) {
    LOG_WRN("Sensor fault detected");
    pmw3610_begin_recovery(data);
    return -EIO;
  }

  // Check MOT bit to ensure valid motion
  if (!(buf[0] & PMW3610_MOTION_MOT)) {
    int irq_active = gpio_pin_get_dt(&config->irq_gpio);
    if (irq_active < 0) {
      LOG_ERR("Failed to read PMW3610 motion IRQ state: %d", irq_active);
      pmw3610_begin_recovery(data);
      return irq_active;
    }

    if (irq_active) {
      data->no_motion_irq_count++;
      LOG_WRN("Motion IRQ remained active without MOT data (%d/%d)",
              data->no_motion_irq_count,
              PMW3610_NO_MOTION_IRQ_RECOVERY_COUNT);
      if (data->no_motion_irq_count >=
          PMW3610_NO_MOTION_IRQ_RECOVERY_COUNT) {
        LOG_ERR("Stuck PMW3610 motion IRQ; restarting sensor");
        pmw3610_begin_recovery(data);
        return -EIO;
      }
    } else {
      data->report_error_count = 0;
      data->no_motion_irq_count = 0;
    }
    return 0;
  }
  data->report_error_count = 0;
  data->no_motion_irq_count = 0;

// 12-bit two's complement value to int16_t
// adapted from
// https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

  int16_t x = TOINT16(
      (buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
  int16_t y = TOINT16(
      (buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

  // A successful SPI transaction can still contain a corrupted motion sample.
  // Reject implausibly large single-frame deltas before they reach the pointer
  // or seed a long inertial tail.
  if (abs(x) >= config->max_motion_delta ||
      abs(y) >= config->max_motion_delta) {
    LOG_WRN("Extreme motion delta detected (x:%d, y:%d), filtering", x, y);
    pmw3610_stop_inertia(data);
    pmw3610_reset_gesture_velocity(data);
    return 0;
  }
  if (config->motion_threshold > 0 &&
      abs(x) <= config->motion_threshold && abs(y) <= config->motion_threshold) {
    LOG_DBG("Drift-sized motion delta filtered (x:%d, y:%d)", x, y);
    return 0;
  }
  pmw3610_apply_low_speed_stabilizer(dev, &x, &y);
  if (x == 0 && y == 0) {
    return 0;
  }
  LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
  int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) +
                    buf[PMW3610_SHUTTER_L_POS];
  if (data->sw_smart_flag && shutter < 45) {
    err = pmw3610_write(dev, 0x32, 0x00);
    if (!err) {
      data->sw_smart_flag = false;
    }
  }
  if (!err && !data->sw_smart_flag && shutter > 45) {
    err = pmw3610_write(dev, 0x32, 0x80);
    if (!err) {
      data->sw_smart_flag = true;
    }
  }
  if (err) {
    LOG_WRN("Smart algorithm write failed; restarting PMW3610: %d", err);
    pmw3610_begin_recovery(data);
    return err;
  }
#endif

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
  // purge accumulated delta, if last sampled had not been reported on last
  // report tick
  if (now - data->last_smp_time >= CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
    data->dx = 0;
    data->dy = 0;
  }
  data->last_smp_time = now;
#endif

  // accumulate delta until report in next iteration
  data->dx += x;
  data->dy += y;

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
  // strict to report inerval
  if (now - data->last_rpt_time < CONFIG_PMW3610_REPORT_INTERVAL_MIN) {
    return 0;
  }
#endif

  // fetch report value
  int16_t rx = (int16_t)CLAMP(data->dx, INT16_MIN, INT16_MAX);
  int16_t ry = (int16_t)CLAMP(data->dy, INT16_MIN, INT16_MAX);
  if (pmw3610_vertical_scroll_direction_is_inverted(dev)) {
    if (config->vertical_scroll_uses_x_axis) {
      rx = rx == INT16_MIN ? INT16_MAX : -rx;
    } else {
      ry = ry == INT16_MIN ? INT16_MAX : -ry;
    }
  }
  if (pmw3610_horizontal_scroll_direction_is_inverted(dev)) {
    if (config->vertical_scroll_uses_x_axis) {
      ry = ry == INT16_MIN ? INT16_MAX : -ry;
    } else {
      rx = rx == INT16_MIN ? INT16_MAX : -rx;
    }
  }
  bool have_x = rx != 0;
  bool have_y = ry != 0;

  if (have_x || have_y) {
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    data->last_rpt_time = now;
#endif
    data->dx = 0;
    data->dy = 0;
    bool inertia_was_enabled = pmw3610_inertial_scroll_is_enabled(dev);
    bool performance_mode_enabled = false;
    int64_t performance_mode_disabled_ms = 0;
    if (inertia_was_enabled) {
      pmw3610_stop_inertia(data);
      k_mutex_lock(&data->inertia_mutex, K_FOREVER);
      performance_mode_enabled = data->performance_mode_enabled;
      performance_mode_disabled_ms = data->performance_mode_disabled_ms;
      k_mutex_unlock(&data->inertia_mutex);
    }

    err = pmw3610_emit_input(dev, rx, ry);
    if (err) {
      LOG_WRN("Input queue full; dropping PMW3610 report: %d", err);
      pmw3610_stop_inertia(data);
      pmw3610_reset_gesture_velocity(data);
      return err;
    }

    if (inertia_was_enabled &&
        pmw3610_inertial_scroll_is_enabled(dev)) {
      pmw3610_update_inertia_from_motion(
          data, config, rx, ry, now, performance_mode_enabled,
          performance_mode_disabled_ms);
    }
  }

  return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob,
                                  struct gpio_callback *cb, uint32_t pins) {
  struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
  const struct device *dev = data->dev;
  pmw3610_set_interrupt(dev, false);
  k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
  struct pixart_data *data =
      CONTAINER_OF(work, struct pixart_data, trigger_work);
  const struct device *dev = data->dev;
  int report_err = pmw3610_report_data(dev);

  // If sensor triggered a fault, data->ready is false.
  // Re-enabling level-triggered IRQ here would cause an infinite loop.
  // The IRQ will be re-enabled securely at the end of the init/recovery
  // sequence.
  if (data->ready) {
    int irq_err = pmw3610_set_interrupt(dev, true);
    if (irq_err) {
      LOG_ERR("Failed to re-enable PMW3610 interrupt after report: %d",
              irq_err);
      pmw3610_begin_recovery(data);
    } else if (report_err) {
      LOG_DBG("PMW3610 report failed but IRQ was restored: %d", report_err);
    }
  }
}

static void pmw3610_inertia_work_callback(struct k_work *work) {
  struct k_work_delayable *delayable = (struct k_work_delayable *)work;
  struct pixart_data *data =
      CONTAINER_OF(delayable, struct pixart_data, inertia_work);
  const struct device *dev = data->dev;
  const struct pixart_config *config = dev->config;

  k_mutex_lock(&data->inertia_mutex, K_FOREVER);

  if (!pmw3610_supports_inertia(dev) ||
      !data->inertial_scroll_enabled ||
      !pmw3610_inertial_layer_active(config)) {
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
    k_mutex_unlock(&data->inertia_mutex);
    return;
  }

  int64_t now = k_uptime_get();
  int64_t elapsed_ms = now - data->inertia_started_ms;
  if (data->inertia_started_ms == 0 ||
      elapsed_ms >= config->inertial_scroll_max_duration_ms) {
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
    k_mutex_unlock(&data->inertia_mutex);
    return;
  }

  data->inertia_vx_q8 =
      (int32_t)(((int64_t)data->inertia_vx_q8 *
                 config->inertial_scroll_decay_basis_points) /
                10000);
  data->inertia_vy_q8 =
      (int32_t)(((int64_t)data->inertia_vy_q8 *
                 config->inertial_scroll_decay_basis_points) /
                10000);

  int64_t remaining_ms =
      config->inertial_scroll_max_duration_ms - elapsed_ms;
  if (config->inertial_scroll_fade_duration_ms > 0 &&
      remaining_ms <= config->inertial_scroll_fade_duration_ms) {
    int64_t next_remaining_ms =
        MAX(remaining_ms - config->inertial_scroll_interval_ms, 0);
    data->inertia_vx_q8 =
        (int32_t)((int64_t)data->inertia_vx_q8 * next_remaining_ms /
                  remaining_ms);
    data->inertia_vy_q8 =
        (int32_t)((int64_t)data->inertia_vy_q8 * next_remaining_ms /
                  remaining_ms);
  }

  int16_t sx = pmw3610_q8_to_step(&data->inertia_rx_q8, data->inertia_vx_q8);
  int16_t sy = pmw3610_q8_to_step(&data->inertia_ry_q8, data->inertia_vy_q8);

  // Continue only while velocity is in the smooth zone (>= 1 step per tick),
  // or while we actually emitted a step this tick from accumulated remainder.
  // Once velocity drops below PMW3610_INERTIA_SCALE the output alternates
  // between 0 and 1 steps per tick, which feels jerky. Stopping as soon as
  // a tick produces no output cleanly ends the scroll without that jitter.
  bool smooth_or_stepped =
      (pmw3610_abs32(data->inertia_vx_q8) >= PMW3610_INERTIA_SCALE ||
       pmw3610_abs32(data->inertia_vy_q8) >= PMW3610_INERTIA_SCALE ||
       sx != 0 || sy != 0);
  bool should_continue =
      pmw3610_inertia_active(data, config) && smooth_or_stepped;
  uint32_t generation = data->inertia_generation;

  /*
   * Input reporting can wait for the ZMK input queue. Do not hold the inertia
   * mutex while emitting, so physical motion and control changes stay prompt.
   */
  k_mutex_unlock(&data->inertia_mutex);
  int emit_err = pmw3610_emit_input(dev, sx, sy);
  k_mutex_lock(&data->inertia_mutex, K_FOREVER);

  if (generation != data->inertia_generation) {
    k_mutex_unlock(&data->inertia_mutex);
    return;
  }
  if (emit_err) {
    LOG_WRN("Input queue full; stopping PMW3610 inertia: %d", emit_err);
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
    k_mutex_unlock(&data->inertia_mutex);
    return;
  }

  if (should_continue) {
    pmw3610_schedule_inertia(data, config);
  } else {
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
  }
  k_mutex_unlock(&data->inertia_mutex);
}

static int pmw3610_init_irq(const struct device *dev) {
  int err;
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;

  // check readiness of irq gpio pin
  if (!device_is_ready(config->irq_gpio.port)) {
    LOG_ERR("IRQ GPIO device not ready");
    return -ENODEV;
  }

  // init the irq pin
  err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
  if (err) {
    LOG_ERR("Cannot configure IRQ GPIO");
    return err;
  }

  // setup and add the irq callback associated
  gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback,
                     BIT(config->irq_gpio.pin));

  err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
  if (err) {
    LOG_ERR("Cannot add IRQ GPIO callback");
  }

  return err;
}

static int pmw3610_init(const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;
  int err;

  if (!spi_is_ready_dt(&config->spi)) {
    LOG_ERR("%s is not ready", config->spi.bus->name);
    return -ENODEV;
  }

  // init device pointer
  data->dev = dev;
  k_mutex_init(&data->spi_mutex);
  k_mutex_init(&data->inertia_mutex);

  // init smart algorithm flag;
  data->sw_smart_flag = false;
  data->inertial_scroll_enabled = pmw3610_supports_inertia(dev);
  data->inertia_vx_q8 = 0;
  data->inertia_vy_q8 = 0;
  data->inertia_rx_q8 = 0;
  data->inertia_ry_q8 = 0;
  data->gesture_vx_q8 = 0;
  data->gesture_vy_q8 = 0;
  data->gesture_last_motion_ms = 0;
  data->inertia_started_ms = 0;
  data->performance_mode_disabled_ms = k_uptime_get();
  pmw3610_reset_micro_motion(data);
  data->vertical_scroll_inverted = false;
  data->horizontal_scroll_inverted = false;
  data->performance_mode_enabled = false;
  data->report_error_count = 0;
  data->no_motion_irq_count = 0;
  data->inertia_generation = 0;
  data->ready = false;
  data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
  data->init_retries = 0;
  // init trigger handler work
  k_work_init(&data->trigger_work, pmw3610_work_callback);
  k_work_init_delayable(&data->inertia_work, pmw3610_inertia_work_callback);

  // init irq routine
  err = pmw3610_init_irq(dev);
  if (err) {
    return err;
  }

  // Setup delayable and non-blocking init jobs, including following steps:
  // 1. power reset
  // 2. upload initial settings
  // 3. other configs like cpi, downshift time, sample time etc.
  // The sensor is ready to work (i.e., data->ready=true after the above steps
  // are finished)
  k_work_init_delayable(&data->init_work, pmw3610_async_init);

  k_work_schedule(&data->init_work,
                  K_MSEC(async_init_delay[data->async_init_step]));

  return err;
}

static int pmw3610_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr,
                            const struct sensor_value *val) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;
  int err;

  if (unlikely(chan != SENSOR_CHAN_ALL)) {
    return -ENOTSUP;
  }

  if (unlikely(!data->ready)) {
    LOG_DBG("Device is not initialized yet");
    return -EBUSY;
  }

  switch ((uint32_t)attr) {
  case PMW3610_ATTR_CPI:
    err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val), config->swap_xy,
                          config->inv_x, config->inv_y);
    break;

  case PMW3610_ATTR_RUN_DOWNSHIFT_TIME:
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                     PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ATTR_REST1_DOWNSHIFT_TIME:
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                     PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ATTR_REST2_DOWNSHIFT_TIME:
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                     PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ATTR_REST1_SAMPLE_TIME:
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                  PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ATTR_REST2_SAMPLE_TIME:
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                  PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ATTR_REST3_SAMPLE_TIME:
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                  PMW3610_SVALUE_TO_TIME(*val));
    break;

  default:
    LOG_ERR("Unknown attribute");
    err = -ENOTSUP;
  }

  return err;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_attr_set,
};

// #if IS_ENABLED(CONFIG_PM_DEVICE)
// static int pmw3610_pm_action(const struct device *dev, enum pm_device_action
// action) {
//     switch (action) {
//     case PM_DEVICE_ACTION_SUSPEND:
//         return pmw3610_set_interrupt(dev, false);
//     case PM_DEVICE_ACTION_RESUME:
//         return pmw3610_set_interrupt(dev, true);
//     default:
//         return -ENOTSUP;
//     }
// }
// #endif // IS_ENABLED(CONFIG_PM_DEVICE)
// PM_DEVICE_DT_INST_DEFINE(n, pmw3610_pm_action);

#define PMW3610_SPI_MODE                                                       \
  (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA |      \
   SPI_TRANSFER_MSB)

#define PMW3610_DECLARE_INERTIAL_LAYERS(n)                                     \
  COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(n), inertial_scroll_layers),        \
              (static const uint8_t inertial_scroll_layers_##n[] =             \
                   DT_PROP(DT_DRV_INST(n), inertial_scroll_layers);),          \
              ())

#define PMW3610_INIT_INERTIAL_LAYERS(n)                                        \
  COND_CODE_1(DT_NODE_HAS_PROP(DT_DRV_INST(n), inertial_scroll_layers),        \
              (.inertial_scroll_layers = inertial_scroll_layers_##n,           \
               .inertial_scroll_layer_count =                                  \
                   ARRAY_SIZE(inertial_scroll_layers_##n),),                   \
              (.inertial_scroll_layers = NULL,                                 \
               .inertial_scroll_layer_count = 0,))

#define PMW3610_VALIDATE_INERTIAL_LAYER(node_id, prop, idx)                    \
  BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, idx) < 32,                        \
               "PMW3610 inertial-scroll-layers values must be below 32");     \
  BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, idx) < ZMK_KEYMAP_LAYERS_LEN,     \
               "PMW3610 inertial-scroll-layers value exceeds keymap layers");

#define PMW3610_VALIDATE_INERTIAL_LAYERS(n)                                    \
  COND_CODE_1(                                                                \
      DT_NODE_HAS_PROP(DT_DRV_INST(n), inertial_scroll_layers),               \
      (DT_FOREACH_PROP_ELEM(DT_DRV_INST(n), inertial_scroll_layers,           \
                            PMW3610_VALIDATE_INERTIAL_LAYER)),                 \
      ())

#define PMW3610_VALIDATE_CONFIG(n)                                              \
  BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), cpi) >= PMW3610_MIN_CPI &&              \
                   DT_PROP(DT_DRV_INST(n), cpi) <= PMW3610_MAX_CPI &&           \
                   DT_PROP(DT_DRV_INST(n), cpi) % 200 == 0,                     \
               "PMW3610 cpi must be 200..3200 in steps of 200");                \
  BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), motion_threshold) <= 1024,               \
               "PMW3610 motion-threshold must be 0..1024");                    \
  BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), max_motion_delta) > 0 &&                 \
                   DT_PROP(DT_DRV_INST(n), max_motion_delta) <= 2048,           \
               "PMW3610 max-motion-delta must be 1..2048");                    \
  BUILD_ASSERT(                                                                \
      !DT_PROP(DT_DRV_INST(n), low_speed_stabilizer) ||                        \
          (DT_PROP(DT_DRV_INST(n), low_speed_stabilizer_threshold) > 0 &&      \
           DT_PROP(DT_DRV_INST(n), low_speed_stabilizer_threshold) <= 16),     \
      "PMW3610 low-speed-stabilizer-threshold must be 1..16");                 \
  BUILD_ASSERT(                                                                \
      !DT_PROP(DT_DRV_INST(n), low_speed_stabilizer) ||                        \
          (DT_PROP(DT_DRV_INST(n), low_speed_stabilizer_timeout_ms) > 0 &&     \
           DT_PROP(DT_DRV_INST(n), low_speed_stabilizer_timeout_ms) <= 1000),  \
      "PMW3610 low-speed-stabilizer-timeout-ms must be 1..1000");              \
  BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), inertial_scroll) ||                     \
                   (DT_PROP(DT_DRV_INST(n), inertial_scroll_decay_pct) > 0 &&   \
                    DT_PROP(DT_DRV_INST(n), inertial_scroll_decay_pct) <= 100), \
               "PMW3610 inertial decay must be 1..100 percent");               \
  BUILD_ASSERT(                                                                \
      DT_PROP(DT_DRV_INST(n), inertial_scroll_decay_basis_points) <= 10000,    \
      "PMW3610 inertial decay basis points must be 0..10000");                 \
  BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), inertial_scroll) ||                     \
                   (DT_PROP(DT_DRV_INST(n), inertial_scroll_interval_ms) > 0 && \
                    DT_PROP(DT_DRV_INST(n), inertial_scroll_interval_ms) <=     \
                        1000),                                                  \
               "PMW3610 inertial interval must be 1..1000 ms");                \
  BUILD_ASSERT(!DT_PROP(DT_DRV_INST(n), inertial_scroll) ||                     \
                   DT_PROP(DT_DRV_INST(n), inertial_scroll_gain_pct) <= 1000,   \
               "PMW3610 inertial gain must be 0..1000 percent");               \
  BUILD_ASSERT(                                                                \
      !DT_PROP(DT_DRV_INST(n), inertial_scroll) ||                             \
          (DT_PROP(DT_DRV_INST(n), inertial_scroll_max_velocity) > 0 &&         \
           DT_PROP(DT_DRV_INST(n), inertial_scroll_max_velocity) <= 1024),      \
      "PMW3610 inertial max velocity must be 1..1024 steps per tick");          \
  BUILD_ASSERT(                                                                \
      !DT_PROP(DT_DRV_INST(n), inertial_scroll) ||                             \
          (DT_PROP(DT_DRV_INST(n), inertial_scroll_max_duration_ms) > 0 &&      \
           DT_PROP(DT_DRV_INST(n), inertial_scroll_max_duration_ms) <= 10000),  \
      "PMW3610 inertial max duration must be 1..10000 ms");                    \
  BUILD_ASSERT(                                                                \
      !DT_PROP(DT_DRV_INST(n), inertial_scroll) ||                             \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_fade_duration_ms) <=         \
              DT_PROP(DT_DRV_INST(n), inertial_scroll_max_duration_ms),        \
      "PMW3610 inertial fade duration must not exceed max duration");         \
  PMW3610_VALIDATE_INERTIAL_LAYERS(n)

#define PMW3610_DEFINE(n)                                                      \
  PMW3610_VALIDATE_CONFIG(n);                                                  \
  PMW3610_DECLARE_INERTIAL_LAYERS(n)                                           \
  static struct pixart_data data##n;                                           \
  static const struct pixart_config config##n = {                              \
      .spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),                     \
      .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                         \
      .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                     \
      .motion_threshold = DT_PROP(DT_DRV_INST(n), motion_threshold),           \
      .max_motion_delta = DT_PROP(DT_DRV_INST(n), max_motion_delta),           \
      .low_speed_stabilizer =                                                  \
          DT_PROP(DT_DRV_INST(n), low_speed_stabilizer),                       \
      .low_speed_stabilizer_threshold =                                        \
          DT_PROP(DT_DRV_INST(n), low_speed_stabilizer_threshold),             \
      .low_speed_stabilizer_timeout_ms =                                       \
          DT_PROP(DT_DRV_INST(n), low_speed_stabilizer_timeout_ms),            \
      .swap_xy = DT_PROP(DT_DRV_INST(n), swap_xy),                             \
      .inv_x = DT_PROP(DT_DRV_INST(n), invert_x),                              \
      .inv_y = DT_PROP(DT_DRV_INST(n), invert_y),                              \
      .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                           \
      .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                   \
      .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                   \
      .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                     \
      .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),   \
      .inertial_scroll = DT_PROP(DT_DRV_INST(n), inertial_scroll),             \
      .scroll_direction_toggle =                                               \
          DT_PROP(DT_DRV_INST(n), scroll_direction_toggle),                    \
      .inertial_scroll_decay_basis_points =                                    \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_decay_basis_points) > 0      \
              ? DT_PROP(DT_DRV_INST(n), inertial_scroll_decay_basis_points)    \
              : DT_PROP(DT_DRV_INST(n), inertial_scroll_decay_pct) * 100,      \
      .inertial_scroll_interval_ms =                                           \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_interval_ms),                \
      .inertial_scroll_threshold =                                             \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_threshold),                  \
      .inertial_scroll_gain_pct =                                              \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_gain_pct),                   \
      .inertial_scroll_max_velocity =                                          \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_max_velocity),               \
      .inertial_scroll_max_duration_ms =                                       \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_max_duration_ms),            \
      .inertial_scroll_fade_duration_ms =                                      \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_fade_duration_ms),           \
      .vertical_scroll_uses_x_axis =                                           \
          DT_PROP(DT_DRV_INST(n), vertical_scroll_uses_x_axis),                 \
      PMW3610_INIT_INERTIAL_LAYERS(n)                                          \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n,           \
                        POST_KERNEL, CONFIG_INPUT_PMW3610_INIT_PRIORITY,       \
                        &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *const pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610, GET_PMW3610_DEV) NULL};

#define PMW3610_DEVICE_COUNT (ARRAY_SIZE(pmw3610_devs) - 1)

void pmw3610_toggle_inertial_scroll_all(void) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    struct pixart_data *data = dev->data;

    if (!pmw3610_supports_inertia(dev)) {
      continue;
    }

    k_mutex_lock(&data->inertia_mutex, K_FOREVER);
    bool enabled = data->inertial_scroll_enabled;
    k_mutex_unlock(&data->inertia_mutex);
    pmw3610_set_inertial_scroll_enabled(dev, !enabled);
  }
}

void pmw3610_set_inertial_scroll_all(bool enabled) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    if (pmw3610_supports_inertia(dev)) {
      pmw3610_set_inertial_scroll_enabled(dev, enabled);
    }
  }
}

void pmw3610_set_vertical_scroll_direction_all(bool inverted) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    if (!pmw3610_supports_scroll_direction(dev)) {
      continue;
    }
    struct pixart_data *data = dev->data;
    k_mutex_lock(&data->inertia_mutex, K_FOREVER);
    if (data->vertical_scroll_inverted == inverted) {
      k_mutex_unlock(&data->inertia_mutex);
      continue;
    }

    data->vertical_scroll_inverted = inverted;
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
    k_mutex_unlock(&data->inertia_mutex);
    k_work_cancel_delayable(&data->inertia_work);
  }
}

void pmw3610_toggle_vertical_scroll_direction_all(void) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    if (!pmw3610_supports_scroll_direction(dev)) {
      continue;
    }
    struct pixart_data *data = dev->data;
    k_mutex_lock(&data->inertia_mutex, K_FOREVER);
    data->vertical_scroll_inverted = !data->vertical_scroll_inverted;
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
    k_mutex_unlock(&data->inertia_mutex);
    k_work_cancel_delayable(&data->inertia_work);
  }
}

void pmw3610_set_horizontal_scroll_direction_all(bool inverted) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    if (!pmw3610_supports_scroll_direction(dev)) {
      continue;
    }
    struct pixart_data *data = dev->data;
    k_mutex_lock(&data->inertia_mutex, K_FOREVER);
    if (data->horizontal_scroll_inverted == inverted) {
      k_mutex_unlock(&data->inertia_mutex);
      continue;
    }

    data->horizontal_scroll_inverted = inverted;
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
    k_mutex_unlock(&data->inertia_mutex);
    k_work_cancel_delayable(&data->inertia_work);
  }
}

void pmw3610_toggle_horizontal_scroll_direction_all(void) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    if (!pmw3610_supports_scroll_direction(dev)) {
      continue;
    }
    struct pixart_data *data = dev->data;
    k_mutex_lock(&data->inertia_mutex, K_FOREVER);
    data->horizontal_scroll_inverted = !data->horizontal_scroll_inverted;
    pmw3610_clear_inertia_locked(data);
    pmw3610_clear_gesture_velocity_locked(data);
    k_mutex_unlock(&data->inertia_mutex);
    k_work_cancel_delayable(&data->inertia_work);
  }
}

static int on_activity_state(const zmk_event_t *eh) {
  struct zmk_activity_state_changed *state_ev =
      as_zmk_activity_state_changed(eh);

  if (!state_ev) {
    LOG_WRN("NO EVENT, leaving early");
    return 0;
  }

  bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 1 : 0;
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    struct pixart_data *data = pmw3610_devs[i]->data;
    if (likely(data->ready)) {
      int err = pmw3610_set_performance(pmw3610_devs[i], enable);
      if (err) {
        LOG_ERR("Failed to change PMW3610 performance state: %d", err);
        pmw3610_begin_recovery(data);
      }
    }
  }

  return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
