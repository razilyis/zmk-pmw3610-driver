#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/util.h>

enum pmw3610_control_kind {
  PMW3610_CONTROL_INERTIA,
  PMW3610_CONTROL_VERTICAL_DIRECTION,
  PMW3610_CONTROL_HORIZONTAL_DIRECTION,
};

enum pmw3610_control_command {
  PMW3610_CONTROL_TOGGLE = 0,
  PMW3610_CONTROL_OFF = 1,
  PMW3610_CONTROL_ON = 2,
  PMW3610_CONTROL_SYNC_STATE = 3,
  PMW3610_CONTROL_SYNC_LAYERS = 4,
};

#define PMW3610_CONTROL_STATE_INERTIA BIT(0)
#define PMW3610_CONTROL_STATE_VERTICAL_DIRECTION BIT(1)
#define PMW3610_CONTROL_STATE_HORIZONTAL_DIRECTION BIT(2)

bool pmw3610_control_get(enum pmw3610_control_kind kind);
int pmw3610_control_convert_toggle(enum pmw3610_control_kind kind,
                                   int32_t *command);
int pmw3610_control_apply(enum pmw3610_control_kind kind, int32_t command,
                          int32_t value);
bool pmw3610_control_remote_layer_active(uint8_t layer);
