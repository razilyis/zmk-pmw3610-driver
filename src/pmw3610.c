/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include "pmw3610.h"
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

static bool pmw3610_supports_inertia(const struct device *dev) {
  const struct pixart_config *config = dev->config;

  return config->inertial_scroll;
}

static int32_t pmw3610_abs32(int32_t value) {
  return value < 0 ? -value : value;
}

static bool pmw3610_inertial_layer_active(const struct pixart_config *config) {
  if (config->inertial_scroll_layer_count == 0) {
    return true;
  }

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  for (size_t i = 0; i < config->inertial_scroll_layer_count; i++) {
    if (zmk_keymap_layer_active(config->inertial_scroll_layers[i])) {
      return true;
    }
  }
#endif

  return false;
}

static void pmw3610_stop_inertia(struct pixart_data *data) {
  k_work_cancel_delayable(&data->inertia_work);
  data->inertia_vx_q8 = 0;
  data->inertia_vy_q8 = 0;
  data->inertia_rx_q8 = 0;
  data->inertia_ry_q8 = 0;
}

static void pmw3610_emit_input(const struct device *dev, int16_t x, int16_t y) {
  const struct pixart_config *config = dev->config;
  bool have_x = x != 0;
  bool have_y = y != 0;

  if (!have_x && !have_y) {
    return;
  }

  if (have_x) {
    input_report(dev, config->evt_type, config->x_input_code, x, !have_y,
                 K_NO_WAIT);
  }
  if (have_y) {
    input_report(dev, config->evt_type, config->y_input_code, y, true,
                 K_NO_WAIT);
  }
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
             config->inertial_scroll_threshold ||
         pmw3610_abs32(data->inertia_rx_q8) >= PMW3610_INERTIA_SCALE ||
         pmw3610_abs32(data->inertia_ry_q8) >= PMW3610_INERTIA_SCALE;
}

static void pmw3610_schedule_inertia(struct pixart_data *data,
                                     const struct pixart_config *config) {
  k_work_reschedule(&data->inertia_work,
                    K_MSEC(config->inertial_scroll_interval_ms));
}

static void pmw3610_update_inertia_from_motion(
    struct pixart_data *data, const struct pixart_config *config, int16_t rx,
    int16_t ry) {
  data->inertia_vx_q8 =
      (rx * config->inertial_scroll_gain_pct * PMW3610_INERTIA_SCALE) / 100;
  data->inertia_vy_q8 =
      (ry * config->inertial_scroll_gain_pct * PMW3610_INERTIA_SCALE) / 100;

  if (pmw3610_inertia_active(data, config)) {
    pmw3610_schedule_inertia(data, config);
  }
}

bool pmw3610_inertial_scroll_is_enabled(const struct device *dev) {
  struct pixart_data *data = dev->data;
  const struct pixart_config *config = dev->config;

  return pmw3610_supports_inertia(dev) && data->inertial_scroll_enabled &&
         pmw3610_inertial_layer_active(config);
}

int pmw3610_set_inertial_scroll_enabled(const struct device *dev,
                                        bool enabled) {
  struct pixart_data *data = dev->data;

  if (!pmw3610_supports_inertia(dev) && enabled) {
    return -ENOTSUP;
  }

  data->inertial_scroll_enabled = enabled && pmw3610_supports_inertia(dev);
  if (!data->inertial_scroll_enabled) {
    pmw3610_stop_inertia(data);
  }

  return 0;
}

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value,
                        uint8_t len) {
  const struct pixart_config *cfg = dev->config;
  const struct spi_buf tx_buf = {.buf = &addr, .len = sizeof(addr)};
  const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
  struct spi_buf rx_buf[] = {
      {
          .buf = NULL,
          .len = sizeof(addr),
      },
      {
          .buf = value,
          .len = len,
      },
  };
  const struct spi_buf_set rx = {.buffers = rx_buf,
                                 .count = ARRAY_SIZE(rx_buf)};

  // Zephyr's SPI API without SPI_HOLD_ON_CS does not guarantee CS is kept low
  // between separate write and read calls. Thus, we MUST use a single
  // transceive operation where the first byte(s) are TX and the subsequent
  // bytes are RX (with NULL TX). The PMW3610 formally requires t_SRAD (address
  // to data delay), but many MCUs handle this naturally via SPI clock gaps or
  // driver overhead. Splitting it risks CS de-assertion which causes fatal
  // runaway tracking. We revert to a single spi_transceive_dt.

  pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                    PMW3610_SPI_CLOCK_CMD_ENABLE);
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  int err = spi_transceive_dt(&cfg->spi, &tx, &rx);

  pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                    PMW3610_SPI_CLOCK_CMD_DISABLE);

  return err;
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
  pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                    PMW3610_SPI_CLOCK_CMD_ENABLE);
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  int err = pmw3610_write_reg(dev, reg, val);
  if (unlikely(err != 0)) {
    return err;
  }

  pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                    PMW3610_SPI_CLOCK_CMD_DISABLE);
  return 0;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi, bool swap_xy,
                           bool inv_x, bool inv_y) {
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

  pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                    PMW3610_SPI_CLOCK_CMD_ENABLE);
  k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

  /* Write data */
  for (size_t i = 0; i < sizeof(data); i++) {
    err = pmw3610_write_reg(dev, addr[i], data[i]);
    if (err) {
      LOG_ERR("Burst write failed on SPI write (data)");
      break;
    }
  }
  pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                    PMW3610_SPI_CLOCK_CMD_DISABLE);

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
              "Backing off for 10 seconds...",
              data->async_init_step);
      // Prevent permanent failure (bricking) but do not spam the SPI bus.
      // Reset retries and back off for a longer period (10 seconds), after
      // which we'll try initializing again.
      data->init_retries = 0;
      data->async_init_step =
          ASYNC_INIT_STEP_POWER_UP; // Restart the entire initialization flow
                                    // from the beginning
      k_work_schedule(&data->init_work, K_SECONDS(10));
    }
  } else {

    data->init_retries = 0; // Reset retries on success
    data->async_init_step++;

    if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
      data->ready = true; // sensor is ready to work
      LOG_INF("PMW3610 initialized");
      pmw3610_set_interrupt(dev, true);
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

#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
  int64_t now = k_uptime_get();
#endif

  int err =
      pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
  if (err) {
    return err;
  }

  // Check MOT bit to ensure valid motion
  if (!(buf[0] & PMW3610_MOTION_MOT)) {
    return 0;
  }

  // Check FAULT bit
  if (unlikely(buf[0] & PMW3610_MOTION_FAULT)) {
    LOG_WRN("Sensor fault detected");
    // Clear accumulated motion to prevent cursor jumps after recovery
    data->dx = 0;
    data->dy = 0;
    pmw3610_stop_inertia(data);
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    data->last_smp_time = 0;
    data->last_rpt_time = 0;
#endif

    // Delegate re-init sequence to async_init_fn to ensure robust recovery
    // and proper retries/delays without blocking or failing silently here.
    data->ready = false;
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    data->init_retries = 0; // Reset retry counter for new init sequence
    k_work_schedule(&data->init_work, K_NO_WAIT);
    return -EIO;
  }

// 12-bit two's complement value to int16_t
// adapted from
// https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

  int16_t x = TOINT16(
      (buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
  int16_t y = TOINT16(
      (buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

  // Filter out potential spikes: 12-bit max is 2047.
  // If delta is extremely large (e.g. > 1024) and MOT was just set, it might be
  // noise.
  if (abs(x) > 1024 || abs(y) > 1024) {
    LOG_WRN("Extreme motion delta detected (x:%d, y:%d), filtering", x, y);
    return 0;
  }
  if (config->motion_threshold > 0 &&
      abs(x) <= config->motion_threshold && abs(y) <= config->motion_threshold) {
    LOG_DBG("Drift-sized motion delta filtered (x:%d, y:%d)", x, y);
    return 0;
  }
  LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_SMART_ALGORITHM
  int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) +
                    buf[PMW3610_SHUTTER_L_POS];
  if (data->sw_smart_flag && shutter < 45) {
    pmw3610_write(dev, 0x32, 0x00);
    data->sw_smart_flag = false;
  }
  if (!data->sw_smart_flag && shutter > 45) {
    pmw3610_write(dev, 0x32, 0x80);
    data->sw_smart_flag = true;
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
  bool have_x = rx != 0;
  bool have_y = ry != 0;

  if (have_x || have_y) {
#if CONFIG_PMW3610_REPORT_INTERVAL_MIN > 0
    data->last_rpt_time = now;
#endif
    data->dx = 0;
    data->dy = 0;
    if (pmw3610_inertial_scroll_is_enabled(dev)) {
      pmw3610_stop_inertia(data);
    }

    pmw3610_emit_input(dev, rx, ry);

    if (pmw3610_inertial_scroll_is_enabled(dev)) {
      pmw3610_update_inertia_from_motion(data, config, rx, ry);
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
  pmw3610_report_data(dev);

  // If sensor triggered a fault, data->ready is false.
  // Re-enabling level-triggered IRQ here would cause an infinite loop.
  // The IRQ will be re-enabled securely at the end of the init/recovery
  // sequence.
  if (data->ready) {
    pmw3610_set_interrupt(dev, true);
  }
}

static void pmw3610_inertia_work_callback(struct k_work *work) {
  struct k_work_delayable *delayable = (struct k_work_delayable *)work;
  struct pixart_data *data =
      CONTAINER_OF(delayable, struct pixart_data, inertia_work);
  const struct device *dev = data->dev;
  const struct pixart_config *config = dev->config;

  if (!pmw3610_inertial_scroll_is_enabled(dev)) {
    pmw3610_stop_inertia(data);
    return;
  }

  data->inertia_vx_q8 =
      (data->inertia_vx_q8 * config->inertial_scroll_decay_pct) / 100;
  data->inertia_vy_q8 =
      (data->inertia_vy_q8 * config->inertial_scroll_decay_pct) / 100;

  int16_t sx = pmw3610_q8_to_step(&data->inertia_rx_q8, data->inertia_vx_q8);
  int16_t sy = pmw3610_q8_to_step(&data->inertia_ry_q8, data->inertia_vy_q8);

  pmw3610_emit_input(dev, sx, sy);

  if (pmw3610_inertia_active(data, config)) {
    pmw3610_schedule_inertia(data, config);
  } else {
    pmw3610_stop_inertia(data);
  }
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

  // init smart algorithm flag;
  data->sw_smart_flag = false;
  data->inertial_scroll_enabled = pmw3610_supports_inertia(dev);
  data->inertia_vx_q8 = 0;
  data->inertia_vy_q8 = 0;
  data->inertia_rx_q8 = 0;
  data->inertia_ry_q8 = 0;

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

#define PMW3610_DEFINE(n)                                                      \
  PMW3610_DECLARE_INERTIAL_LAYERS(n)                                           \
  static struct pixart_data data##n;                                           \
  static const struct pixart_config config##n = {                              \
      .spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),                     \
      .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                         \
      .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                     \
      .motion_threshold = DT_PROP(DT_DRV_INST(n), motion_threshold),           \
      .swap_xy = DT_PROP(DT_DRV_INST(n), swap_xy),                             \
      .inv_x = DT_PROP(DT_DRV_INST(n), invert_x),                              \
      .inv_y = DT_PROP(DT_DRV_INST(n), invert_y),                              \
      .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                           \
      .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                   \
      .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                   \
      .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                     \
      .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),   \
      .inertial_scroll = DT_PROP(DT_DRV_INST(n), inertial_scroll),             \
      .inertial_scroll_decay_pct =                                             \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_decay_pct),                  \
      .inertial_scroll_interval_ms =                                           \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_interval_ms),                \
      .inertial_scroll_threshold =                                             \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_threshold),                  \
      .inertial_scroll_gain_pct =                                              \
          DT_PROP(DT_DRV_INST(n), inertial_scroll_gain_pct),                   \
      PMW3610_INIT_INERTIAL_LAYERS(n)                                          \
  };                                                                           \
  DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n,           \
                        POST_KERNEL, CONFIG_INPUT_PMW3610_INIT_PRIORITY,       \
                        &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610, GET_PMW3610_DEV)};

void pmw3610_toggle_inertial_scroll_all(void) {
  for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
    const struct device *dev = pmw3610_devs[i];
    struct pixart_data *data = dev->data;

    if (!pmw3610_supports_inertia(dev)) {
      continue;
    }

    pmw3610_set_inertial_scroll_enabled(dev, !data->inertial_scroll_enabled);
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
  for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
    struct pixart_data *data = pmw3610_devs[i]->data;
    if (likely(data->ready)) {
      pmw3610_set_performance(pmw3610_devs[i], enable);
    }
  }

  return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
