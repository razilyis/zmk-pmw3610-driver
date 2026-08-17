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
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* device data structure */
struct pixart_data {
    const struct device          *dev;
    int64_t                      dx;
    int64_t                      dy;
#if CONFIG_PMW3610_ALT_REPORT_INTERVAL_MIN > 0
    int64_t                      last_smp_time;
    int64_t                      last_rpt_time;
#endif
    bool                         sw_smart_flag; // for pmw3610 smart algorithm

    struct gpio_callback         irq_gpio_cb; // motion pin irq callback
    struct k_work                trigger_work; // realtrigger job

    struct k_work_delayable      init_work; // the work structure for delayable init steps
    int                          async_init_step;

    struct k_work_delayable      inertia_work;
    int32_t                      inertia_x;
    int32_t                      inertia_y;
    int32_t                      inertia_accum_x;
    int32_t                      inertia_accum_y;
    int64_t                      inertia_start_time;
    int32_t                      gesture_vx;
    int32_t                      gesture_vy;
    int64_t                      last_motion_time;

    bool                         ready; // whether init is finished successfully
    int                          err; // error code during async init
};

// device config data structure
struct pixart_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
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
    uint16_t inertial_scroll_decay_pct;
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
