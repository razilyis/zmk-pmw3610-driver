/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_pmw3610_inertia_toggle

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>

#include "pmw3610.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
  ARG_UNUSED(binding);
  ARG_UNUSED(event);

  pmw3610_toggle_inertial_scroll_all();
  return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
  ARG_UNUSED(binding);
  ARG_UNUSED(event);

  return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw3610_inertia_toggle_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

#define PMW3610_INERTIA_TOGGLE_INST(n)                                         \
  BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,              \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                 \
                          &behavior_pmw3610_inertia_toggle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_INERTIA_TOGGLE_INST)

#endif
