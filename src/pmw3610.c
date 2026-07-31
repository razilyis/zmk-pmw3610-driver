/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610_alt

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_ALT_LOG_LEVEL);

K_THREAD_STACK_DEFINE(pmw3610_work_q_stack, CONFIG_PMW3610_ALT_WORKQUEUE_STACK_SIZE);
static struct k_work_q pmw3610_work_q;

static int pmw3610_work_queue_init(void) {
    static const struct k_work_queue_config queue_config = {
        .name = "PMW3610 Work Queue",
    };

    k_work_queue_start(&pmw3610_work_q, pmw3610_work_q_stack,
                       K_THREAD_STACK_SIZEOF(pmw3610_work_q_stack),
                       CONFIG_PMW3610_ALT_WORKQUEUE_PRIORITY, &queue_config);
    return 0;
}

SYS_INIT(pmw3610_work_queue_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
BUILD_ASSERT(CONFIG_INPUT_PMW3610_INIT_PRIORITY > CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
             "PMW3610 device init must run after its work queue init");

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registes like cpi and donwshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10 + CONFIG_PMW3610_ALT_INIT_POWER_UP_EXTRA_DELAY_MS, // >10ms needed
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200, // 150 us required, test shows too short,
                                       // also power-up reset is added in this step, thus using 50 ms
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,  // 10 ms required in spec,
                                       // test shows too short,
                                       // especially when integrated with display,
                                       // > 50ms is needed
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Function definitions //////////

static int pmw3610_set_interrupt(const struct device *dev, const bool en);

static int pmw3610_read_unlocked(const struct device *dev, uint8_t addr, uint8_t *value,
                                 uint8_t len) {
	const struct pixart_config *cfg = dev->config;
	const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf rx_buf[] = {
		{ .buf = NULL, .len = sizeof(addr), },
		{ .buf = value, .len = len, },
	};
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
    struct pixart_data *data = dev->data;
    int err;

    k_mutex_lock(&data->spi_mutex, K_FOREVER);
    err = pmw3610_read_unlocked(dev, addr, value, len);
    k_mutex_unlock(&data->spi_mutex);
    return err;
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
	return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
	const struct pixart_config *cfg = dev->config;
	uint8_t write_buf[] = {addr | SPI_WRITE_BIT, value};
	const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf), };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1, };
	return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
    struct pixart_data *data = dev->data;
    int err;
    int disable_err;

    k_mutex_lock(&data->spi_mutex, K_FOREVER);
	err = pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (err) {
        k_mutex_unlock(&data->spi_mutex);
        return err;
    }
	k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));

    err = pmw3610_write_reg(dev, reg, val);
    disable_err =
        pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    k_mutex_unlock(&data->spi_mutex);

    return err ? err : disable_err;
}

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi, 
                           bool swap_xy, bool inv_x, bool inv_y) {
    /* Set resolution with CPI step of 200 cpi
     * 0x1: 200 cpi (minimum cpi)
     * 0x2: 400 cpi
     * 0x3: 600 cpi
     * :
     */

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI) || (cpi % 200 != 0)) {
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

#if IS_ENABLED(CONFIG_PMW3610_ALT_SWAP_XY)
    value |= (1 << 7);
#else
    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_X)
    value |= (1 << 6);
#else
    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_Y)
    value |= (1 << 5);
#else
    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }
#endif

    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    /* set the cpi */
    uint8_t addr[] = {0x7F, PMW3610_REG_RES_STEP, 0x7F};
    uint8_t data[] = {0xFF, value,                0x00};

    struct pixart_data *driver_data = dev->data;
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
static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range [%u, %u]", sample_time, mintime, maxtime);
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
// NOTE: The unit of run-mode downshift is related to pos mode rate, which is hard coded to be 4 ms
// The pos-mode rate is configured in pmw3610_async_init_configure
static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                      * 8 * pos-rate (fixed to 4ms)
         */
        maxtime = 8160; // 32 * 255;
        mintime = 32; // hard-coded in pmw3610_async_init_configure
        break;

    case PMW3610_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3610_REG_RUN_DOWNSHIFT
         *                        * 16 * Rest1_sample_period (default 40 ms)
         */
        maxtime = 255 * 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        break;

    case PMW3610_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3610_REG_REST2_DOWNSHIFT
         *                        * 128 * Rest2 rate (default 100 ms)
         */
        maxtime = 255 * 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
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
        uint8_t perf;
        if (config->force_awake_4ms_mode) {
            perf = 0x0d; // RUN RATE @ 4ms
        } else {
            // reset bit[3..0] to 0x0 (normal operation)
            perf = 0x00; // RUN RATE @ 8ms
        }

        if (enabled) {
            perf |= 0xF0; // set bit[3..0] to 0xF (force awake)
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
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    struct pixart_data *data = dev->data;
    int ret;

    k_mutex_lock(&data->spi_mutex, K_FOREVER);
	ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    k_mutex_unlock(&data->spi_mutex);
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
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
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
        err = pmw3610_set_cpi(dev, config->cpi, config->swap_xy, config->inv_x, config->inv_y);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,
                                         CONFIG_PMW3610_ALT_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT,
                                         CONFIG_PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT,
                                         CONFIG_PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE,
                                      CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE,
                                      CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS);
    }

    if (!err) {
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE,
                                      CONFIG_PMW3610_ALT_REST3_SAMPLE_TIME_MS);
    }

    if (err) {
        LOG_ERR("Config the sensor failed");
        return err;
    }

    return 0;
}

static void pmw3610_begin_recovery(struct pixart_data *data) {
    if (!data->ready) {
        return;
    }

    data->ready = false;
    (void)pmw3610_set_interrupt(data->dev, false);
    data->dx = 0;
    data->dy = 0;
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    data->last_smp_time = 0;
    data->last_rpt_time = 0;
#endif
    data->report_error_count = 0;
    data->no_motion_irq_count = 0;
    data->no_motion_irq_since_ms = 0;
    data->input_retry_pending = false;
    data->input_retry_since_ms = 0;
    data->input_frame_open = false;
    data->irq_recheck_pending = false;
    data->init_retries = 0;
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    k_work_reschedule_for_queue(&pmw3610_work_q, &data->init_work,
                                K_MSEC(CONFIG_PMW3610_ALT_RECOVERY_DELAY_MS));
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        if (data->init_retries < PMW3610_INIT_STEP_RETRY_COUNT) {
            data->init_retries++;
            LOG_ERR("PMW3610 initialization failed in step %d, retrying (%d/%d)",
                    data->async_init_step, data->init_retries,
                    PMW3610_INIT_STEP_RETRY_COUNT);
            k_work_reschedule_for_queue(&pmw3610_work_q, &data->init_work, K_MSEC(100));
        } else {
            LOG_ERR("PMW3610 initialization failed in step %d; restarting in %d ms",
                    data->async_init_step, CONFIG_PMW3610_ALT_INIT_RETRY_BACKOFF_MS);
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
            data->ready = true; // sensor is ready to work
            LOG_INF("PMW3610 initialized");
            data->err = pmw3610_set_interrupt(dev, true);
            if (data->err) {
                LOG_ERR("Failed to enable PMW3610 interrupt; restarting init");
                data->ready = false;
                data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
                k_work_reschedule_for_queue(&pmw3610_work_q, &data->init_work, K_MSEC(100));
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
    k_timeout_t timeout = K_MSEC(CONFIG_PMW3610_ALT_INPUT_REPORT_TIMEOUT_MS);
    int first_err = 0;

    *x_sent = false;
    *y_sent = false;

    /*
     * An earlier X event may already be queued without sync. Close that frame
     * before sending any newer X value so an axis cannot be duplicated. While
     * this state is active, pmw3610_report_data() retries the saved matching Y
     * value without reading a newer sensor sample.
     */
    if (data->input_frame_open) {
        int err;
        if (have_y) {
            err = input_report(dev, config->evt_type, config->y_input_code, y, true, timeout);
            if (!err) {
                *y_sent = true;
                data->input_frame_open = false;
            }
        } else {
            err = input_report(dev, config->evt_type, config->x_input_code, 0, true, timeout);
            if (!err) {
                data->input_frame_open = false;
            }
        }
        return err;
    }

    if (have_x) {
        int err =
            input_report(dev, config->evt_type, config->x_input_code, x, !have_y, timeout);
        if (err) {
            first_err = err;
        } else {
            *x_sent = true;
        }
    }
    if (have_y) {
        int err = input_report(dev, config->evt_type, config->y_input_code, y, true, timeout);
        if (err) {
            if (!first_err) {
                first_err = err;
            }
            if (have_x && *x_sent) {
                int flush_err =
                    input_report(dev, config->evt_type, config->x_input_code, 0, true, timeout);
                if (flush_err) {
                    data->input_frame_open = true;
                }
            }
        } else {
            *y_sent = true;
        }
    }

    return first_err;
}

static void pmw3610_limit_pending_motion(struct pixart_data *data,
                                         const struct pixart_config *config) {
    int64_t limit = config->max_motion_delta;

    data->dx = CLAMP(data->dx, -limit, limit);
    data->dy = CLAMP(data->dy, -limit, limit);
}

static int pmw3610_retry_pending_input(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int64_t retry_now = k_uptime_get();

    if (retry_now - data->input_retry_since_ms >=
        CONFIG_PMW3610_ALT_INPUT_RETRY_TIMEOUT_MS) {
        LOG_WRN("Dropping stale PMW3610 motion after input congestion");
        int flush_err = 0;
        if (data->input_frame_open) {
            flush_err =
                input_report(dev, config->evt_type, config->x_input_code, 0, true, K_NO_WAIT);
        }
        data->dx = 0;
        data->dy = 0;
        data->input_frame_open = flush_err != 0;
        data->input_retry_pending = data->input_frame_open;
        data->input_retry_since_ms = data->input_frame_open ? retry_now : 0;
        return flush_err ? -EAGAIN : 0;
    }

    int16_t rx = (int16_t)CLAMP(data->dx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(data->dy, INT16_MIN, INT16_MAX);
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
#endif

	int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
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
    // LOG_HEXDUMP_DBG(buf, PMW3610_BURST_SIZE, "buf");

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);

    if (x >= config->max_motion_delta || x <= -(int32_t)config->max_motion_delta ||
        y >= config->max_motion_delta || y <= -(int32_t)config->max_motion_delta) {
        LOG_WRN("Extreme motion delta filtered (x:%d, y:%d)", x, y);
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
        data->last_smp_time = now;
#endif
        return 0;
    }
    LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_ALT_SMART_ALGORITHM
    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8) 
                    + buf[PMW3610_SHUTTER_L_POS];
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
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (!data->input_retry_pending &&
        now - data->last_smp_time >= CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN) {
        data->dx = 0;
        data->dy = 0;
    }
    data->last_smp_time = now;
#endif

    // accumulate delta until report in next iteration
    data->dx += x;
    data->dy += y;

#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - data->last_rpt_time < CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN) {
        return 0;
    }
#endif

    // fetch report value
    int16_t rx = (int16_t)CLAMP(data->dx, INT16_MIN, INT16_MAX);
    int16_t ry = (int16_t)CLAMP(data->dy, INT16_MIN, INT16_MAX);
    bool have_x = rx != 0;
    bool have_y = ry != 0;

    if (have_x || have_y) {
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
        data->input_retry_pending =
            data->input_frame_open || data->dx != 0 || data->dy != 0;
        if (data->input_retry_pending && !data->input_retry_since_ms) {
            data->input_retry_since_ms = k_uptime_get();
        } else if (!data->input_retry_pending) {
            data->input_retry_since_ms = 0;
        }
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
        if ((have_x && x_sent) || (have_y && y_sent)) {
            data->last_rpt_time = now;
        }
#endif
        if (err) {
            LOG_WRN("Failed to queue PMW3610 input: %d", err);
        }
        if (err || data->input_retry_pending) {
            return -EAGAIN;
        }
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    int err = pmw3610_set_interrupt(dev, false);
    if (err) {
        LOG_ERR("Failed to disable PMW3610 interrupt: %d", err);
        return;
    }

    err = k_work_reschedule_for_queue(&pmw3610_work_q, &data->trigger_work, K_NO_WAIT);
    if (err < 0) {
        LOG_ERR("Failed to submit PMW3610 work: %d", err);
        if (pmw3610_set_interrupt(dev, true)) {
            pmw3610_begin_recovery(data);
        }
    }
}

static void pmw3610_work_callback(struct k_work *work) {
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
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
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct pixart_data *data =
        CONTAINER_OF(delayable, struct pixart_data, performance_work);

    if (!data->ready) {
        return;
    }

    int err = pmw3610_set_performance(
        data->dev, atomic_get(&data->performance_requested) != 0);
    if (err) {
        LOG_ERR("Failed to change PMW3610 performance state: %d", err);
        pmw3610_begin_recovery(data);
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
    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

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

    // init smart algorithm flag;
    data->sw_smart_flag = false;
    data->dx = 0;
    data->dy = 0;
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    data->last_smp_time = 0;
    data->last_rpt_time = 0;
#endif
    data->ready = false;
    data->input_retry_pending = false;
    data->input_retry_since_ms = 0;
    data->input_frame_open = false;
    data->irq_recheck_pending = false;
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    data->init_retries = 0;
    data->report_error_count = 0;
    data->no_motion_irq_count = 0;
    data->no_motion_irq_since_ms = 0;
    atomic_set(&data->performance_requested, 1);

    // init trigger handler work
    k_work_init_delayable(&data->trigger_work, pmw3610_work_callback);
    k_work_init_delayable(&data->performance_work,
                          pmw3610_performance_work_callback);

    // init irq routine
    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. upload initial settings
    // 3. other configs like cpi, downshift time, sample time etc.
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3610_async_init);

    k_work_schedule_for_queue(&pmw3610_work_q, &data->init_work,
                              K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

static int pmw3610_alt_attr_set(const struct device *dev, enum sensor_channel chan,
                            enum sensor_attribute attr, const struct sensor_value *val) {
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
    case PMW3610_ALT_ATTR_CPI:
        err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val),
                              config->swap_xy, config->inv_x, config->inv_y);
        break;

    case PMW3610_ALT_ATTR_RUN_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST1_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST2_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST1_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST2_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;

    case PMW3610_ALT_ATTR_REST3_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
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

// #if IS_ENABLED(CONFIG_PM_DEVICE)
// static int pmw3610_pm_action(const struct device *dev, enum pm_device_action action) {
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

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                        SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PMW3610_DEFINE(n)                                                                          \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), cpi) >= PMW3610_MIN_CPI &&                               \
                     DT_PROP(DT_DRV_INST(n), cpi) <= PMW3610_MAX_CPI &&                            \
                     DT_PROP(DT_DRV_INST(n), cpi) % 200 == 0,                                      \
                 "PMW3610 cpi must be 200..3200 in steps of 200");                                 \
    BUILD_ASSERT(DT_PROP(DT_DRV_INST(n), max_motion_delta) > 0 &&                                  \
                     DT_PROP(DT_DRV_INST(n), max_motion_delta) <= 2047,                            \
                 "PMW3610 max-motion-delta must be 1..2047");                                     \
    static struct pixart_data data##n;                                                             \
    static const struct pixart_config config##n = {                                                \
		.spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),		                               \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                                       \
        .max_motion_delta = DT_PROP(DT_DRV_INST(n), max_motion_delta),                             \
        .swap_xy = DT_PROP(DT_DRV_INST(n), swap_xy),                                               \
        .inv_x = DT_PROP(DT_DRV_INST(n), invert_x),                                                \
        .inv_y = DT_PROP(DT_DRV_INST(n), invert_y),                                                \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                                             \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                                     \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                                     \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                                       \
        .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_INPUT_PMW3610_INIT_PRIORITY, &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)


#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610_alt, GET_PMW3610_DEV)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);

    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }

    bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 1 : 0;
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        struct pixart_data *data = pmw3610_devs[i]->data;
        atomic_set(&data->performance_requested, enable);
        k_work_reschedule_for_queue(&pmw3610_work_q, &data->performance_work,
                                    K_NO_WAIT);
    }

    return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
