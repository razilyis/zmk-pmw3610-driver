/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610_alt

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util_macro.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/keymap.h>

#include "pmw3610.h"
#include "pmw3610_control.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_ALT_LOG_LEVEL);

#define PMW3610_INERTIA_FP_SHIFT 8
#define PMW3610_GESTURE_TIMEOUT_MS 80
#define PMW3610_DEFAULT_SAMPLE_RATE_MS 8
#define PMW3610_PERF_SAMPLE_RATE_MS 4

static const uint32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 1,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 1,
    [ASYNC_INIT_STEP_CHECK_OB1] = 10,
    [ASYNC_INIT_STEP_CONFIGURE] = 1,
};

static struct k_work_q pmw3610_work_q;
static K_THREAD_STACK_DEFINE(pmw3610_work_q_stack,
                             CONFIG_PMW3610_ALT_WORKQUEUE_STACK_SIZE);

static int pmw3610_work_queue_init(void) {
  k_work_queue_start(&pmw3610_work_q, pmw3610_work_q_stack,
                     K_THREAD_STACK_SIZEOF(pmw3610_work_q_stack),
                     CONFIG_PMW3610_ALT_WORKQUEUE_PRIORITY, NULL);
  k_thread_name_set(&pmw3610_work_q.thread, "pmw3610_work_q");
  return 0;
}

SYS_INIT(pmw3610_work_queue_init, POST_KERNEL,
         CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(
    const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

static int pmw3610_set_interrupt(const struct device *dev, const bool en);
static int pmw3610_write_reg(const struct device *dev, uint8_t addr,
                             uint8_t value);
static void pmw3610_inertia_work_callback(struct k_work *work);

static inline int32_t pmw3610_scale_fp(int32_t val, uint32_t pct) {
  return (int32_t)(((int64_t)val * (int64_t)pct) / 100);
}

static inline int32_t pmw3610_scale_basis_points(int32_t val, uint32_t bp) {
  return (int32_t)(((int64_t)val * (int64_t)bp) / 10000);
}

static inline int32_t pmw3610_decay_velocity(
    int32_t val, const struct pixart_config *config) {
  if (config->inertial_scroll_decay_basis_points > 0) {
    return pmw3610_scale_basis_points(
        val, config->inertial_scroll_decay_basis_points);
  }
  return pmw3610_scale_fp(val, config->inertial_scroll_decay_pct);
}

static bool pmw3610_layer_matches(uint8_t layer, const uint8_t *layers,
                                  size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (layers[i] == layer) {
      return true;
    }
  }
  return false;
}

static bool pmw3610_layer_mask_matches(uint32_t mask, const uint8_t *layers,
                                       size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (layers[i] < 32 && (mask & BIT(layers[i]))) {
      return true;
    }
  }
  return false;
}

static bool pmw3610_supports_inertia(const struct device *dev) {
  const struct pixart_config *config = dev->config;
  return config->inertial_scroll;
}

bool pmw3610_inertial_scroll_is_enabled(const struct device *dev) {
  const struct pixart_config *config = dev->config;

  if (!config->inertial_scroll) {
    return false;
  }

  if (!pmw3610_control_inertia_enabled()) {
    return false;
  }

  if (config->inertial_scroll_layer_count == 0) {
    return true;
  }

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  uint8_t active_layer = zmk_keymap_highest_layer_active();
  return pmw3610_layer_matches(active_layer, config->inertial_scroll_layers,
                               config->inertial_scroll_layer_count);
#else
  uint32_t active_mask = pmw3610_control_get_active_layer_mask();
  return pmw3610_layer_mask_matches(active_mask, config->inertial_scroll_layers,
                                    config->inertial_scroll_layer_count);
#endif
}

bool pmw3610_vertical_scroll_direction_is_inverted(const struct device *dev) {
  const struct pixart_config *config = dev->config;
  if (!config->inertial_scroll && !config->scroll_direction_toggle) {
    return false;
  }
  if (config->inertial_scroll_layer_count > 0 &&
      !pmw3610_inertial_scroll_is_enabled(dev)) {
    return false;
  }
  return pmw3610_control_vertical_scroll_inverted();
}

bool pmw3610_horizontal_scroll_direction_is_inverted(
    const struct device *dev) {
  const struct pixart_config *config = dev->config;
  if (!config->inertial_scroll && !config->scroll_direction_toggle) {
    return false;
  }
  if (config->inertial_scroll_layer_count > 0 &&
      !pmw3610_inertial_scroll_is_enabled(dev)) {
    return false;
  }
  return pmw3610_control_horizontal_scroll_inverted();
}

static void pmw3610_stop_inertia(struct pixart_data *data) {
  k_work_cancel_delayable(&data->inertia_work);
  k_mutex_lock(&data->inertia_mutex, K_FOREVER);
  data->inertia_x = 0;
  data->inertia_y = 0;
  data->inertia_accum_x = 0;
  data->inertia_accum_y = 0;
  data->inertia_start_time = 0;
  k_mutex_unlock(&data->inertia_mutex);
}

static void pmw3610_reset_gesture_velocity(struct pixart_data *data) {
  data->gesture_vx = 0;
  data->gesture_vy = 0;
  data->last_motion_time = 0;
}

static void pmw3610_update_inertia_from_motion(
    struct pixart_data *data, const struct pixart_config *config,
    int16_t dx, int16_t dy, int64_t now) {
  int64_t dt_ms = (data->last_motion_time > 0) ? (now - data->last_motion_time) : 0;
  data->last_motion_time = now;

  if (dt_ms > PMW3610_GESTURE_TIMEOUT_MS) {
    pmw3610_reset_gesture_velocity(data);
    dt_ms = 0;
  }

  uint32_t norm_period_ms = PMW3610_DEFAULT_SAMPLE_RATE_MS;
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
  if (CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > norm_period_ms) {
    norm_period_ms = CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN;
  }
#endif

  int32_t inst_vx = ((int32_t)dx << PMW3610_INERTIA_FP_SHIFT);
  int32_t inst_vy = ((int32_t)dy << PMW3610_INERTIA_FP_SHIFT);

  if (dt_ms > 0 && dt_ms != norm_period_ms) {
    inst_vx = (int32_t)(((int64_t)inst_vx * (int64_t)norm_period_ms) / dt_ms);
    inst_vy = (int32_t)(((int64_t)inst_vy * (int64_t)norm_period_ms) / dt_ms);
  }

  if (data->gesture_vx == 0 && data->gesture_vy == 0) {
    data->gesture_vx = inst_vx;
    data->gesture_vy = inst_vy;
  } else {
    data->gesture_vx = (data->gesture_vx * 3 + inst_vx) / 4;
    data->gesture_vy = (data->gesture_vy * 3 + inst_vy) / 4;
  }

  int32_t init_vx = pmw3610_scale_fp(data->gesture_vx, config->inertial_scroll_gain_pct);
  int32_t init_vy = pmw3610_scale_fp(data->gesture_vy, config->inertial_scroll_gain_pct);

  int32_t max_vel = (int32_t)config->inertial_scroll_max_velocity << PMW3610_INERTIA_FP_SHIFT;
  if (max_vel > 0) {
    init_vx = CLAMP(init_vx, -max_vel, max_vel);
    init_vy = CLAMP(init_vy, -max_vel, max_vel);
  }

  int32_t thresh = (int32_t)config->inertial_scroll_threshold;
  if (abs(init_vx) >= thresh || abs(init_vy) >= thresh) {
    k_mutex_lock(&data->inertia_mutex, K_FOREVER);
    data->inertia_x = init_vx;
    data->inertia_y = init_vy;
    data->inertia_accum_x = 0;
    data->inertia_accum_y = 0;
    data->inertia_start_time = now;
    k_mutex_unlock(&data->inertia_mutex);

    k_work_reschedule_for_queue(
        &pmw3610_work_q, &data->inertia_work,
        K_MSEC(config->inertial_scroll_interval_ms));
  }
}

static void pmw3610_inertia_work_callback(struct k_work *work) {
  struct k_work_delayable *delayable = (struct k_work_delayable *)work;
  struct pixart_data *data =
      CONTAINER_OF(delayable, struct pixart_data, inertia_work);
  const struct device *dev = data->dev;
  const struct pixart_config *config = dev->config;

  k_mutex_lock(&data->inertia_mutex, K_FOREVER);

  int32_t vx = data->inertia_x;
  int32_t vy = data->inertia_y;

  if (vx == 0 && vy == 0) {
    k_mutex_unlock(&data->inertia_mutex);
    return;
  }

  int64_t now = k_uptime_get();
  int64_t elapsed_ms = now - data->inertia_start_time;

  if (config->inertial_scroll_max_duration_ms > 0 &&
      elapsed_ms >= config->inertial_scroll_max_duration_ms) {
    data->inertia_x = 0;
    data->inertia_y = 0;
    k_mutex_unlock(&data->inertia_mutex);
    return;
  }

  vx = pmw3610_decay_velocity(vx, config);
  vy = pmw3610_decay_velocity(vy, config);

  if (config->inertial_scroll_fade_duration_ms > 0 &&
      config->inertial_scroll_max_duration_ms > config->inertial_scroll_fade_duration_ms) {
    int64_t fade_start =
        config->inertial_scroll_max_duration_ms - config->inertial_scroll_fade_duration_ms;
    if (elapsed_ms > fade_start) {
      int64_t remaining = config->inertial_scroll_max_duration_ms - elapsed_ms;
      if (remaining <= 0) {
        vx = 0;
        vy = 0;
      } else {
        vx = (int32_t)(((int64_t)vx * remaining) / config->inertial_scroll_fade_duration_ms);
        vy = (int32_t)(((int64_t)vy * remaining) / config->inertial_scroll_fade_duration_ms);
      }
    }
  }

  int32_t thresh = (int32_t)config->inertial_scroll_threshold;
  if (abs(vx) < thresh && abs(vy) < thresh) {
    data->inertia_x = 0;
    data->inertia_y = 0;
    k_mutex_unlock(&data->inertia_mutex);
    return;
  }

  data->inertia_x = vx;
  data->inertia_y = vy;

  data->inertia_accum_x += vx;
  data->inertia_accum_y += vy;

  int16_t sx = (int16_t)(data->inertia_accum_x >> PMW3610_INERTIA_FP_SHIFT);
  int16_t sy = (int16_t)(data->inertia_accum_y >> PMW3610_INERTIA_FP_SHIFT);

  if (sx != 0) {
    data->inertia_accum_x -= ((int32_t)sx << PMW3610_INERTIA_FP_SHIFT);
  }
  if (sy != 0) {
    data->inertia_accum_y -= ((int32_t)sy << PMW3610_INERTIA_FP_SHIFT);
  }

  k_mutex_unlock(&data->inertia_mutex);

  if (sx != 0 || sy != 0) {
    k_timeout_t timeout = K_MSEC(CONFIG_PMW3610_ALT_INPUT_REPORT_TIMEOUT_MS);
    if (sx != 0) {
      input_report(dev, config->evt_type, config->x_input_code, sx, (sy == 0), timeout);
    }
    if (sy != 0) {
      input_report(dev, config->evt_type, config->y_input_code, sy, true, timeout);
    }
  }

  k_work_reschedule_for_queue(
      &pmw3610_work_q, &data->inertia_work,
      K_MSEC(config->inertial_scroll_interval_ms));
}

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value,
                        uint8_t len) {
  const struct pixart_config *cfg = dev->config;
  uint8_t write_buf[] = {addr & SPI_READ_BIT};
  const struct spi_buf tx_buf = {
      .buf = write_buf,
      .len = sizeof(write_buf),
  };
  const struct spi_buf_set tx = {
      .buffers = &tx_buf,
      .count = 1,
  };
  struct spi_buf rx_buf[] = {
      {
          .buf = NULL,
          .len = sizeof(write_buf),
      },
      {
          .buf = value,
          .len = len,
      },
  };
  const struct spi_buf_set rx = {
      .buffers = rx_buf,
      .count = ARRAY_SIZE(rx_buf),
  };
  struct spi_config read_config = cfg->spi.config;
  read_config.operation &= ~SPI_HOLD_ON_CS;

  int err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                              PMW3610_SPI_CLOCK_CMD_ENABLE);
  if (err) {
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
  int err = spi_write_dt(&cfg->spi, &tx);
  if (!err) {
    k_busy_wait(T_SWW_DELAY_US);
  }
  return err;
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
  int err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                              PMW3610_SPI_CLOCK_CMD_ENABLE);
  if (err) {
    return err;
  }
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  err = pmw3610_write_reg(dev, reg, val);
  int disable_err =
      pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                        PMW3610_SPI_CLOCK_CMD_DISABLE);

  return err ? err : disable_err;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi,
                           bool swap_xy, bool inv_x, bool inv_y) {
  if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
    LOG_ERR("CPI value %u out of range", cpi);
    return -EINVAL;
  }

  uint8_t value = 0x00;
  int err = 0;

  LOG_INF("Setting cpi: %d", cpi);
  uint8_t cpi_val = cpi / 200;
  value = (value & 0xE0) | (cpi_val & 0x1F);

  LOG_INF("Setting axis swap_xy: %s inv_x: %s inv_y: %s",
          swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

#if IS_ENABLED(CONFIG_PMW3610_ALT_SWAP_XY) || IS_ENABLED(CONFIG_PMW3610_SWAP_XY)
  value |= (1 << 7);
#else
  if (swap_xy) {
    value |= (1 << 7);
  } else {
    value &= ~(1 << 7);
  }
#endif

#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_X) || IS_ENABLED(CONFIG_PMW3610_INVERT_X)
  value |= (1 << 6);
#else
  if (inv_x) {
    value |= (1 << 6);
  } else {
    value &= ~(1 << 6);
  }
#endif

#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_Y) || IS_ENABLED(CONFIG_PMW3610_INVERT_Y)
  value |= (1 << 5);
#else
  if (inv_y) {
    value |= (1 << 5);
  } else {
    value &= ~(1 << 5);
  }
#endif

  LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);
  uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
  uint8_t data[] = {0xFF, value, 0x00};

  err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                          PMW3610_SPI_CLOCK_CMD_ENABLE);
  if (err) {
    return err;
  }
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  for (size_t i = 0; i < ARRAY_SIZE(addr); i++) {
    err = pmw3610_write_reg(dev, addr[i], data[i]);
    if (err) {
      LOG_ERR("Failed to set CPI: %d", err);
      break;
    }
  }

  int disable_err =
      pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                        PMW3610_SPI_CLOCK_CMD_DISABLE);

  return err ? err : disable_err;
}

static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr,
                                   uint32_t sample_time) {
  if (sample_time > PMW3610_MAX_SAMPLE_TIME) {
    LOG_ERR("Sample time %u out of range", sample_time);
    return -EINVAL;
  }
  uint8_t value = (sample_time / 10) - 1;
  return pmw3610_write(dev, reg_addr, value);
}

static int pmw3610_set_downshift_time(const struct device *dev,
                                      uint8_t reg_addr, uint32_t time) {
  uint8_t pos;
  uint32_t max_time;
  uint32_t mul;

  switch (reg_addr) {
  case PMW3610_REG_RUN_DOWNSHIFT:
    pos = 0;
    mul = 8;
    max_time = PMW3610_RUN_DOWNSHIFT_MULT * (pos + 1);
    break;
  case PMW3610_REG_REST1_DOWNSHIFT:
    pos = 4;
    mul = 8;
    max_time = PMW3610_REST1_DOWNSHIFT_MULT * (pos + 1);
    break;
  case PMW3610_REG_REST2_DOWNSHIFT:
    pos = 8;
    mul = 16;
    max_time = PMW3610_REST2_DOWNSHIFT_MULT * (pos + 1);
    break;
  default:
    LOG_ERR("Not supported");
    return -ENOTSUP;
  }

  if (time > max_time) {
    LOG_ERR("Downshift time %u out of range", time);
    return -EINVAL;
  }

  uint8_t value = ((time / mul) - 1) & 0xFF;
  return pmw3610_write(dev, reg_addr, value);
}

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
  uint8_t value;
  int err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
  if (err) {
    LOG_ERR("Failed to read performance register");
    return err;
  }

  if (enabled) {
    value |= PMW3610_PERFORMANCE_FORCED_REST_DISABLED;
    value |= PMW3610_PERFORMANCE_OPERATION_MODE_NORMAL;
  } else {
    value &= ~PMW3610_PERFORMANCE_FORCED_REST_DISABLED;
  }

  return pmw3610_write(dev, PMW3610_REG_PERFORMANCE, value);
}

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
  const struct pixart_config *config = dev->config;
  int ret = gpio_pin_interrupt_configure_dt(
      &config->irq_gpio, en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
  if (ret < 0) {
    LOG_ERR("Failed to set interrupt (en:%d)", en);
  }
  return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
  return pmw3610_write(dev, PMW3610_REG_POWER_UP_RESET,
                       PMW3610_POWERUP_CMD_RESET);
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
                                     PMW3610_RUN_DOWNSHIFT_TIME_MS);
  }
  if (!err) {
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                     PMW3610_REST1_DOWNSHIFT_TIME_MS);
  }
  if (!err) {
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                     PMW3610_REST2_DOWNSHIFT_TIME_MS);
  }
  if (!err) {
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                  PMW3610_REST1_SAMPLE_TIME_MS);
  }
  if (!err) {
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                  PMW3610_REST2_SAMPLE_TIME_MS);
  }
  if (!err) {
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                  PMW3610_REST3_SAMPLE_TIME_MS);
  }

  if (err) {
    LOG_ERR("Config the sensor failed");
    return err;
  }
  return 0;
}

static void pmw3610_begin_recovery(struct pixart_data *data) {
  const struct device *dev = data->dev;
  data->ready = false;
  data->init_retries = 0;
  data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
  data->dx = 0;
  data->dy = 0;
  data->input_retry_pending = false;
  data->input_retry_since_ms = 0;
  data->input_frame_open = false;
  data->irq_recheck_pending = false;
  data->no_motion_irq_count = 0;
  data->no_motion_irq_since_ms = 0;
  pmw3610_stop_inertia(data);
  pmw3610_reset_gesture_velocity(data);

  (void)pmw3610_set_interrupt(dev, false);
  k_work_cancel_delayable(&data->trigger_work);
  k_work_reschedule_for_queue(&pmw3610_work_q, &data->init_work,
                              K_MSEC(CONFIG_PMW3610_ALT_RECOVERY_DELAY_MS));
}

static void pmw3610_async_init(struct k_work *work) {
  struct k_work_delayable *delayable = (struct k_work_delayable *)work;
  struct pixart_data *data =
      CONTAINER_OF(delayable, struct pixart_data, init_work);
  const struct device *dev = data->dev;

  data->err = async_init_fn[data->async_init_step](dev);
  if (data->err) {
    if (data->init_retries < PMW3610_INIT_STEP_RETRY_COUNT) {
      data->init_retries++;
      LOG_ERR("PMW3610 initialization failed in step %d, retrying (%d/%d)",
              data->async_init_step, data->init_retries,
              PMW3610_INIT_STEP_RETRY_COUNT);
      k_work_reschedule_for_queue(&pmw3610_work_q, &data->init_work,
                                  K_MSEC(100));
    } else {
      LOG_ERR("PMW3610 initialization failed in step %d; restarting in %d ms",
              data->async_init_step,
              CONFIG_PMW3610_ALT_INIT_RETRY_BACKOFF_MS);
      data->init_retries = 0;
      data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
      k_work_reschedule_for_queue(
          &pmw3610_work_q, &data->init_work,
          K_MSEC(CONFIG_PMW3610_ALT_INIT_RETRY_BACKOFF_MS));
    }
  } else {
    data->init_retries = 0;
    data->async_init_step++;

    if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
      data->ready = true;
      LOG_INF("PMW3610 initialized");
      data->err = pmw3610_set_interrupt(dev, true);
      if (data->err) {
        LOG_ERR("Failed to enable PMW3610 interrupt; restarting init");
        data->ready = false;
        data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
        k_work_reschedule_for_queue(&pmw3610_work_q, &data->init_work,
                                    K_MSEC(100));
      } else {
        k_work_reschedule_for_queue(&pmw3610_work_q, &data->performance_work,
                                    K_NO_WAIT);
      }
    } else {
      k_work_reschedule_for_queue(
          &pmw3610_work_q, &data->init_work,
          K_MSEC(async_init_delay[data->async_init_step]));
    }
  }
}

static int pmw3610_emit_input(const struct device *dev, int16_t x, int16_t y,
                              bool *x_sent, bool *y_sent) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;
  bool have_x = x != 0;
  bool have_y = y != 0;
  bool x_sync =
      !have_y || (IS_ENABLED(CONFIG_ZMK_SPLIT) &&
                  !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL));
  k_timeout_t timeout = K_MSEC(CONFIG_PMW3610_ALT_INPUT_REPORT_TIMEOUT_MS);
  int first_err = 0;

  *x_sent = false;
  *y_sent = false;

  if (data->input_frame_open) {
    int err;
    if (have_y) {
      err = input_report(dev, config->evt_type, config->y_input_code, y, true,
                         timeout);
      if (!err) {
        *y_sent = true;
        data->input_frame_open = false;
      }
    } else {
      err = input_report(dev, config->evt_type, config->x_input_code, 0, true,
                         timeout);
      if (!err) {
        data->input_frame_open = false;
      }
    }
    return err;
  }

  if (have_x) {
    int err = input_report(dev, config->evt_type, config->x_input_code, x,
                           x_sync, timeout);
    if (err) {
      first_err = err;
    } else {
      *x_sent = true;
      if (!x_sync) {
        data->input_frame_open = true;
      }
    }
  }

  if (have_y && (!have_x || *x_sent)) {
    int err = input_report(dev, config->evt_type, config->y_input_code, y, true,
                           timeout);
    if (err) {
      if (!first_err) {
        first_err = err;
      }
    } else {
      *y_sent = true;
      data->input_frame_open = false;
    }
  }

  return first_err;
}

static void pmw3610_limit_pending_motion(struct pixart_data *data,
                                         const struct pixart_config *config) {
  int32_t limit = (int32_t)config->max_report_delta;
  if (data->dx > limit) {
    data->dx = limit;
  } else if (data->dx < -limit) {
    data->dx = -limit;
  }
  if (data->dy > limit) {
    data->dy = limit;
  } else if (data->dy < -limit) {
    data->dy = -limit;
  }
}

static int16_t pmw3610_bounded_report_delta(
    int64_t delta, const struct pixart_config *config) {
  int32_t limit = (int32_t)config->max_report_delta;
  if (delta > limit) {
    return (int16_t)limit;
  }
  if (delta < -limit) {
    return (int16_t)-limit;
  }
  return (int16_t)delta;
}

static int pmw3610_retry_pending_input(const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;
  int64_t retry_now = k_uptime_get();

  if (CONFIG_PMW3610_ALT_INPUT_RETRY_TIMEOUT_MS > 0 &&
      data->input_retry_since_ms &&
      retry_now - data->input_retry_since_ms >=
          CONFIG_PMW3610_ALT_INPUT_RETRY_TIMEOUT_MS) {
    LOG_WRN("Input queue retry timed out; dropping pending motion");
    data->input_retry_pending = false;
    data->input_retry_since_ms = 0;
    data->dx = 0;
    data->dy = 0;
    if (data->input_frame_open) {
      k_timeout_t timeout =
          K_MSEC(CONFIG_PMW3610_ALT_INPUT_REPORT_TIMEOUT_MS);
      int flush_err = input_report(dev, config->evt_type,
                                   config->x_input_code, 0, true, timeout);
      data->input_frame_open = flush_err != 0;
      data->input_retry_pending = data->input_frame_open;
      data->input_retry_since_ms = data->input_frame_open ? retry_now : 0;
      return flush_err ? -EAGAIN : 0;
    }
  }

  int16_t rx = pmw3610_bounded_report_delta(data->dx, config);
  int16_t ry = pmw3610_bounded_report_delta(data->dy, config);
  bool x_sent;
  bool y_sent;
  int err = pmw3610_emit_input(dev, rx, ry, &x_sent, &y_sent);

  if (x_sent) {
    data->dx -= rx;
  }
  if (y_sent) {
    data->dy -= ry;
  }
  pmw3610_limit_pending_motion(data, config);

  data->input_retry_pending =
      data->input_frame_open || data->dx != 0 || data->dy != 0;
  if (!data->input_retry_pending) {
    data->input_retry_since_ms = 0;
    return 0;
  }

  return err ? err : -EAGAIN;
}

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

static int pmw3610_report_data(const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;
  uint8_t buf[PMW3610_BURST_SIZE];

  if (unlikely(!data->ready)) {
    LOG_WRN("Device is not initialized yet");
    return -EBUSY;
  }

  if (data->input_retry_pending) {
    int retry_err = pmw3610_retry_pending_input(dev);
    if (retry_err || !data->irq_recheck_pending) {
      return retry_err;
    }
  }

#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
  int64_t now = k_uptime_get();
#else
  int64_t now = k_uptime_get();
#endif

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

  if (unlikely(buf[0] & PMW3610_MOTION_FAULT)) {
    LOG_ERR("PMW3610 fault detected; restarting sensor");
    pmw3610_begin_recovery(data);
    return -EIO;
  }

  if (!(buf[0] & PMW3610_MOTION_MOT)) {
    int irq_active = gpio_pin_get_dt(&config->irq_gpio);
    if (irq_active < 0) {
      pmw3610_begin_recovery(data);
      return irq_active;
    }
    if (irq_active) {
      int64_t irq_now = k_uptime_get();
      if (data->no_motion_irq_count < UINT8_MAX) {
        data->no_motion_irq_count++;
      }
      if (!data->no_motion_irq_since_ms) {
        data->no_motion_irq_since_ms = irq_now;
      }
      if (data->no_motion_irq_count >= PMW3610_NO_MOTION_IRQ_RECOVERY_COUNT &&
          irq_now - data->no_motion_irq_since_ms >=
              CONFIG_PMW3610_ALT_STUCK_IRQ_TIME_MS) {
        LOG_ERR("Stuck PMW3610 motion IRQ; restarting sensor");
        pmw3610_begin_recovery(data);
        return -EIO;
      }
      data->irq_recheck_pending = true;
      return -EAGAIN;
    } else {
      data->no_motion_irq_count = 0;
      data->no_motion_irq_since_ms = 0;
      data->irq_recheck_pending = false;
    }
    data->report_error_count = 0;
    return 0;
  }

  data->report_error_count = 0;
  data->no_motion_irq_count = 0;
  data->no_motion_irq_since_ms = 0;
  data->irq_recheck_pending = false;

  int16_t x = TOINT16(
      (buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
  int16_t y = TOINT16(
      (buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

  if (x >= config->max_motion_delta || x <= -(int32_t)config->max_motion_delta ||
      y >= config->max_motion_delta || y <= -(int32_t)config->max_motion_delta) {
    LOG_WRN("Extreme motion delta filtered (x:%d, y:%d)", x, y);
    data->dx = 0;
    data->dy = 0;
    pmw3610_stop_inertia(data);
    pmw3610_reset_gesture_velocity(data);
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    data->last_smp_time = now;
#endif
    return 0;
  }

  if (config->motion_threshold > 0 &&
      x <= config->motion_threshold && x >= -(int32_t)config->motion_threshold &&
      y <= config->motion_threshold && y >= -(int32_t)config->motion_threshold) {
    LOG_DBG("Drift-sized motion delta filtered (x:%d, y:%d)", x, y);
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    if (data->dx != 0 || data->dy != 0) {
      if (now - data->last_smp_time >= CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN) {
        data->dx = 0;
        data->dy = 0;
      } else if (now - data->last_rpt_time >=
                 CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN) {
        goto emit_pending_motion;
      }
    }
#endif
    return 0;
  }

  LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_ALT_SMART_ALGORITHM
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
    pmw3610_begin_recovery(data);
    return err;
  }
#endif

#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
  if (!data->input_retry_pending &&
      now - data->last_smp_time >= CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN) {
    data->dx = 0;
    data->dy = 0;
  }
  data->last_smp_time = now;
#endif

  data->dx += x;
  data->dy += y;

#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
  if (!data->input_retry_pending &&
      now - data->last_rpt_time < CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN) {
    return 0;
  }
emit_pending_motion:
#endif

  int16_t rx = pmw3610_bounded_report_delta(data->dx, config);
  int16_t ry = pmw3610_bounded_report_delta(data->dy, config);

  if (pmw3610_vertical_scroll_direction_is_inverted(dev)) {
    if (config->vertical_scroll_uses_x_axis) {
      rx = (rx == INT16_MIN) ? INT16_MAX : -rx;
    } else {
      ry = (ry == INT16_MIN) ? INT16_MAX : -ry;
    }
  }
  if (pmw3610_horizontal_scroll_direction_is_inverted(dev)) {
    if (config->vertical_scroll_uses_x_axis) {
      ry = (ry == INT16_MIN) ? INT16_MAX : -ry;
    } else {
      rx = (rx == INT16_MIN) ? INT16_MAX : -rx;
    }
  }

  bool have_x = rx != 0;
  bool have_y = ry != 0;

  if (have_x || have_y) {
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    data->last_rpt_time = now;
#endif
    bool inertia_active = pmw3610_inertial_scroll_is_enabled(dev);
    if (inertia_active) {
      pmw3610_stop_inertia(data);
    }

    bool x_sent;
    bool y_sent;
    err = pmw3610_emit_input(dev, rx, ry, &x_sent, &y_sent);
    if (x_sent) {
      data->dx -= rx;
    }
    if (y_sent) {
      data->dy -= ry;
    }
    pmw3610_limit_pending_motion(data, config);
    if (err) {
      data->input_retry_pending = true;
      data->input_retry_since_ms = now;
      LOG_WRN("Input queue full; retrying PMW3610 report: %d", err);
      pmw3610_stop_inertia(data);
      pmw3610_reset_gesture_velocity(data);
      return -EAGAIN;
    }
    data->input_retry_pending = false;
    data->input_retry_since_ms = 0;

    if (inertia_active && pmw3610_inertial_scroll_is_enabled(dev)) {
      pmw3610_update_inertia_from_motion(data, config, rx, ry, now);
    }
  }

  return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob,
                                  struct gpio_callback *cb, uint32_t pins) {
  struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
  const struct device *dev = data->dev;
  int err = pmw3610_set_interrupt(dev, false);
  if (err) {
    LOG_ERR("Failed to disable PMW3610 interrupt: %d", err);
    return;
  }

  err = k_work_reschedule_for_queue(&pmw3610_work_q, &data->trigger_work,
                                    K_NO_WAIT);
  if (err < 0) {
    LOG_ERR("Failed to submit PMW3610 work: %d", err);
    if (pmw3610_set_interrupt(dev, true)) {
      pmw3610_begin_recovery(data);
    }
  }
}

static void pmw3610_work_callback(struct k_work *work) {
  struct k_work_delayable *delayable = (struct k_work_delayable *)work;
  struct pixart_data *data =
      CONTAINER_OF(delayable, struct pixart_data, trigger_work);
  const struct device *dev = data->dev;
  int report_err = pmw3610_report_data(dev);

  if (data->ready && report_err == -EAGAIN &&
      (data->input_retry_pending || data->irq_recheck_pending)) {
    int retry_err = k_work_reschedule_for_queue(
        &pmw3610_work_q, &data->trigger_work,
        K_MSEC(PMW3610_IRQ_RECHECK_DELAY_MS));
    if (retry_err < 0) {
      LOG_ERR("Failed to schedule PMW3610 retry: %d", retry_err);
      pmw3610_begin_recovery(data);
    }
    return;
  }

  if (data->ready) {
    int irq_err = pmw3610_set_interrupt(dev, true);
    if (irq_err) {
      LOG_ERR("Failed to re-enable PMW3610 interrupt: %d", irq_err);
      pmw3610_begin_recovery(data);
    } else if (report_err) {
      LOG_DBG("PMW3610 report failed but IRQ was restored: %d", report_err);
    }
  }
}

static void pmw3610_performance_work_callback(struct k_work *work) {
  struct k_work_delayable *delayable = (struct k_work_delayable *)work;
  struct pixart_data *data =
      CONTAINER_OF(delayable, struct pixart_data, performance_work);
  const struct device *dev = data->dev;
  const struct pixart_config *config = dev->config;

  if (!data->ready) {
    return;
  }

  bool req = (bool)atomic_get(&data->performance_requested);

  if (config->force_awake) {
    if (req) {
      if (config->force_awake_4ms_mode) {
        (void)pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                      PMW3610_PERF_SAMPLE_RATE_MS);
      }
      (void)pmw3610_set_performance(dev, true);
    } else {
      (void)pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                    PMW3610_REST1_SAMPLE_TIME_MS);
      (void)pmw3610_set_performance(dev, false);
    }
  } else {
    (void)pmw3610_set_performance(dev, req);
  }
}

static int pmw3610_init_irq(const struct device *dev) {
  int err = 0;
  const struct pixart_config *config = dev->config;
  struct pixart_data *data = dev->data;

  if (!gpio_is_ready_dt(&config->irq_gpio)) {
    LOG_ERR("IRQ GPIO device not ready");
    return -ENODEV;
  }

  err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
  if (err) {
    LOG_ERR("Failed to configure IRQ GPIO pin");
    return err;
  }

  gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback,
                     BIT(config->irq_gpio.pin));

  err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
  if (err) {
    LOG_ERR("Failed to add IRQ callback");
    return err;
  }

  return 0;
}

static int pmw3610_init(const struct device *dev) {
  const struct pixart_config *config = dev->config;
  struct pixart_data *data = dev->data;
  int err;

  data->dev = dev;
  data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
  data->ready = false;
  data->report_error_count = 0;
  data->no_motion_irq_count = 0;
  data->no_motion_irq_since_ms = 0;
  atomic_set(&data->performance_requested, 1);

  k_mutex_init(&data->inertia_mutex);
  k_work_init_delayable(&data->trigger_work, pmw3610_work_callback);
  k_work_init_delayable(&data->performance_work,
                        pmw3610_performance_work_callback);
  k_work_init_delayable(&data->inertia_work, pmw3610_inertia_work_callback);

  err = pmw3610_init_irq(dev);
  if (err) {
    return err;
  }

  k_work_init_delayable(&data->init_work, pmw3610_async_init);
  k_work_schedule_for_queue(
      &pmw3610_work_q, &data->init_work,
      K_MSEC(async_init_delay[data->async_init_step]));

  return err;
}

static int pmw3610_alt_attr_set(const struct device *dev,
                                enum sensor_channel chan,
                                enum sensor_attribute attr,
                                const struct sensor_value *val) {
  const struct pixart_config *config = dev->config;
  struct pixart_data *data = dev->data;
  int err;

  if (unlikely(chan != SENSOR_CHAN_ALL)) {
    return -ENOTSUP;
  }

  if (unlikely(!data->ready)) {
    LOG_DBG("Device is not initialized yet");
    return -EBUSY;
  }

  switch ((uint32_t)attr) {
  case PMW3610_ALT_ATTR_CPI:
    err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val), config->swap_xy,
                          config->inv_x, config->inv_y);
    break;

  case PMW3610_ALT_ATTR_RUN_DOWNSHIFT_TIME:
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                     PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ALT_ATTR_REST1_DOWNSHIFT_TIME:
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                     PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ALT_ATTR_REST2_DOWNSHIFT_TIME:
    err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                     PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ALT_ATTR_REST1_SAMPLE_TIME:
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                  PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ALT_ATTR_REST2_SAMPLE_TIME:
    err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                  PMW3610_SVALUE_TO_TIME(*val));
    break;

  case PMW3610_ALT_ATTR_REST3_SAMPLE_TIME:
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
    .attr_set = pmw3610_alt_attr_set,
};

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
  BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), max_report_delta) > 0 &&                 \
                   DT_PROP(DT_DRV_INST(n), max_report_delta) <= 2047,           \
               "PMW3610 max-report-delta must be 1..2047");                    \
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
      .spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, T_CS_HOLD_DELAY_US),    \
      .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                         \
      .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                     \
      .motion_threshold = DT_PROP(DT_DRV_INST(n), motion_threshold),           \
      .max_motion_delta = DT_PROP(DT_DRV_INST(n), max_motion_delta),           \
      .max_report_delta = DT_PROP(DT_DRV_INST(n), max_report_delta),           \
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
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3610_alt)
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610_alt, GET_PMW3610_DEV)
#endif
#if DT_HAS_COMPAT_STATUS_OKAY(pixart_pmw3610)
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610, GET_PMW3610_DEV)
#endif
    NULL
};

#define PMW3610_DEVICE_COUNT (ARRAY_SIZE(pmw3610_devs) - 1)

void pmw3610_toggle_inertial_scroll_all(void) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    struct pixart_data *data = dev->data;
    if (pmw3610_supports_inertia(dev)) {
      pmw3610_stop_inertia(data);
      pmw3610_reset_gesture_velocity(data);
    }
  }
}

void pmw3610_toggle_vertical_scroll_direction_all(void) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    struct pixart_data *data = dev->data;
    if (pmw3610_supports_inertia(dev)) {
      pmw3610_stop_inertia(data);
      pmw3610_reset_gesture_velocity(data);
    }
  }
}

void pmw3610_toggle_horizontal_scroll_direction_all(void) {
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    const struct device *dev = pmw3610_devs[i];
    struct pixart_data *data = dev->data;
    if (pmw3610_supports_inertia(dev)) {
      pmw3610_stop_inertia(data);
      pmw3610_reset_gesture_velocity(data);
    }
  }
}

void pmw3610_invert_scroll_all(bool invert) {
  ARG_UNUSED(invert);
  pmw3610_toggle_vertical_scroll_direction_all();
}

void pmw3610_invert_horizontal_scroll_all(bool invert) {
  ARG_UNUSED(invert);
  pmw3610_toggle_horizontal_scroll_direction_all();
}

void pmw3610_set_vertical_scroll_direction_all(bool inverted) {
  ARG_UNUSED(inverted);
  pmw3610_toggle_vertical_scroll_direction_all();
}

void pmw3610_set_horizontal_scroll_direction_all(bool inverted) {
  ARG_UNUSED(inverted);
  pmw3610_toggle_horizontal_scroll_direction_all();
}

void pmw3610_set_inertial_scroll_all(bool enabled) {
  ARG_UNUSED(enabled);
  pmw3610_toggle_inertial_scroll_all();
}

static int on_activity_state(const zmk_event_t *eh) {
  struct zmk_activity_state_changed *state_ev =
      as_zmk_activity_state_changed(eh);

  if (!state_ev) {
    LOG_WRN("NO EVENT, leaving early");
    return 0;
  }

  bool enable = (state_ev->state == ZMK_ACTIVITY_ACTIVE);
  for (size_t i = 0; i < PMW3610_DEVICE_COUNT; i++) {
    struct pixart_data *data = pmw3610_devs[i]->data;
    atomic_set(&data->performance_requested, enable ? 1 : 0);
    k_work_reschedule_for_queue(&pmw3610_work_q, &data->performance_work,
                                K_NO_WAIT);
  }

  return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
