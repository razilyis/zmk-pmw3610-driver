#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct pixart_data {
    const struct device          *dev;
    bool                         sw_smart_flag; // for pmw3610 smart algorithm
    int64_t                      dx;
    int64_t                      dy;
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    int64_t                      last_smp_time;
    int64_t                      last_rpt_time;
#endif

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work_delayable      trigger_work; // motion and retry job
    struct k_work_delayable      performance_work;
    struct k_mutex               spi_mutex; // serialize sensor command sequences

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;
    uint8_t                      init_retries;
    uint8_t                      report_error_count;
    uint8_t                      no_motion_irq_count;
    int64_t                      no_motion_irq_since_ms;
    int64_t                      input_retry_since_ms;
    atomic_t                     performance_requested;

    bool                         ready; // whether init is finished successfully
    bool                         input_retry_pending;
    bool                         input_frame_open;
    bool                         irq_recheck_pending;
    int                          err; // error code during async init
};

// device config data structure
struct pixart_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
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
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
