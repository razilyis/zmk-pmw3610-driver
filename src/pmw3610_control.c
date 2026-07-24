/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "pmw3610_control.h"

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/keymap.h>

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#if IS_ENABLED(CONFIG_PMW3610)
#include "pmw3610.h"
#endif

#define PMW3610_HAS_INERTIA_BEHAVIOR                                           \
  DT_HAS_COMPAT_STATUS_OKAY(zmk_behavior_pmw3610_inertia_toggle)
#define PMW3610_HAS_VERTICAL_BEHAVIOR                                          \
  DT_HAS_COMPAT_STATUS_OKAY(zmk_behavior_pmw3610_scroll_direction_toggle)
#define PMW3610_HAS_HORIZONTAL_BEHAVIOR                                        \
  DT_HAS_COMPAT_STATUS_OKAY(                                                   \
      zmk_behavior_pmw3610_horizontal_scroll_direction_toggle)
#define PMW3610_HAS_CONTROL_BEHAVIOR                                           \
  (PMW3610_HAS_INERTIA_BEHAVIOR || PMW3610_HAS_VERTICAL_BEHAVIOR ||            \
   PMW3610_HAS_HORIZONTAL_BEHAVIOR)

#if PMW3610_HAS_CONTROL_BEHAVIOR

LOG_MODULE_REGISTER(pmw3610_control, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(ZMK_KEYMAP_LAYERS_LEN <= 32,
             "PMW3610 split layer synchronization supports up to 32 layers");

static atomic_t control_state =
    ATOMIC_INIT(PMW3610_CONTROL_STATE_INERTIA);
static atomic_t remote_layer_mask = ATOMIC_INIT(BIT(0));

#if IS_ENABLED(CONFIG_SETTINGS)
static struct k_work_delayable settings_save_work;
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT) &&                                           \
    IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) &&                             \
    PMW3610_HAS_INERTIA_BEHAVIOR
#include <zmk/split/central.h>
#include <zmk/split/transport/central.h>

BUILD_ASSERT(ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT <= 32,
             "PMW3610 split synchronization supports up to 32 peripherals");

#define PMW3610_SYNC_IDLE_INTERVAL K_SECONDS(5)
#define PMW3610_SYNC_CHANGE_DELAY K_MSEC(100)

extern const struct zmk_split_transport_central *active_transport;

static struct k_work_delayable split_sync_work;
static atomic_t dirty_sources =
    ATOMIC_INIT(BIT_MASK(ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT));
static uint32_t connected_sources;

static void pmw3610_control_schedule_split_sync(k_timeout_t delay) {
  k_work_reschedule(&split_sync_work, delay);
}

static uint32_t pmw3610_control_local_layer_mask(void) {
  uint32_t mask = 0;

  for (uint8_t layer = 0; layer < ZMK_KEYMAP_LAYERS_LEN; layer++) {
    if (zmk_keymap_layer_active(layer)) {
      mask |= BIT(layer);
    }
  }

  return mask;
}

static int pmw3610_control_send_sync(uint8_t source) {
  struct zmk_behavior_binding binding = {
      .behavior_dev = DEVICE_DT_NAME(
          DT_COMPAT_GET_ANY_STATUS_OKAY(
              zmk_behavior_pmw3610_inertia_toggle)),
      .param1 = PMW3610_CONTROL_SYNC_STATE,
      .param2 = atomic_get(&control_state),
  };
  struct zmk_behavior_binding_event event = {
      .position = 0,
      .timestamp = k_uptime_get(),
  };

  int err = zmk_split_central_invoke_behavior(source, &binding, event, true);
  if (err) {
    return err;
  }

  binding.param1 = PMW3610_CONTROL_SYNC_LAYERS;
  binding.param2 = (int32_t)pmw3610_control_local_layer_mask();
  return zmk_split_central_invoke_behavior(source, &binding, event, true);
}

static void pmw3610_control_split_sync_work(struct k_work *work) {
  ARG_UNUSED(work);

  uint32_t current_sources = 0;
  if (active_transport && active_transport->api &&
      active_transport->api->get_available_source_ids) {
    uint8_t sources[ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT];
    int count =
        active_transport->api->get_available_source_ids(sources);
    if (count > 0) {
      for (int i = 0; i < count; i++) {
        if (sources[i] < 32) {
          current_sources |= BIT(sources[i]);
        }
      }
    }
  }

  uint32_t newly_connected = current_sources & ~connected_sources;
  uint32_t sources_to_sync =
      current_sources & ((uint32_t)atomic_get(&dirty_sources) |
                         newly_connected);

  for (uint8_t source = 0;
       source < ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT; source++) {
    if (!(sources_to_sync & BIT(source))) {
      continue;
    }

    int err = pmw3610_control_send_sync(source);
    if (err) {
      LOG_WRN("Failed to synchronize PMW3610 state to split source %u: %d",
              source, err);
      continue;
    }
    /*
     * A newly connected peripheral can be reported as connected before
     * behavior discovery has completely settled. Keep it dirty for one more
     * polling cycle so the full state is sent again.
     */
    if (!(newly_connected & BIT(source))) {
      atomic_and(&dirty_sources, ~BIT(source));
    }
  }

  connected_sources = current_sources;
  pmw3610_control_schedule_split_sync(PMW3610_SYNC_IDLE_INTERVAL);
}

static void pmw3610_control_mark_split_dirty(void) {
  atomic_or(&dirty_sources,
            BIT_MASK(ZMK_SPLIT_CENTRAL_PERIPHERAL_COUNT));
  pmw3610_control_schedule_split_sync(PMW3610_SYNC_CHANGE_DELAY);
}
#else
static void pmw3610_control_mark_split_dirty(void) {}
#endif

static uint32_t pmw3610_control_bit(enum pmw3610_control_kind kind) {
  switch (kind) {
  case PMW3610_CONTROL_INERTIA:
    return PMW3610_CONTROL_STATE_INERTIA;
  case PMW3610_CONTROL_VERTICAL_DIRECTION:
    return PMW3610_CONTROL_STATE_VERTICAL_DIRECTION;
  case PMW3610_CONTROL_HORIZONTAL_DIRECTION:
    return PMW3610_CONTROL_STATE_HORIZONTAL_DIRECTION;
  default:
    return 0;
  }
}

bool pmw3610_control_get(enum pmw3610_control_kind kind) {
  uint32_t bit = pmw3610_control_bit(kind);
  return bit && (atomic_get(&control_state) & bit);
}

static void pmw3610_control_apply_to_sensors(void) {
#if IS_ENABLED(CONFIG_PMW3610)
  pmw3610_set_inertial_scroll_all(
      pmw3610_control_get(PMW3610_CONTROL_INERTIA));
  pmw3610_set_vertical_scroll_direction_all(
      pmw3610_control_get(PMW3610_CONTROL_VERTICAL_DIRECTION));
  pmw3610_set_horizontal_scroll_direction_all(
      pmw3610_control_get(PMW3610_CONTROL_HORIZONTAL_DIRECTION));
#endif
}

#if IS_ENABLED(CONFIG_SETTINGS)
static void pmw3610_control_save_work(struct k_work *work) {
  ARG_UNUSED(work);
  uint8_t state = (uint8_t)atomic_get(&control_state);
  int err = settings_save_one("pmw3610/control", &state, sizeof(state));
  if (err) {
    LOG_WRN("Failed to save PMW3610 control state: %d", err);
  }
}

static void pmw3610_control_schedule_save(void) {
  k_work_reschedule(&settings_save_work,
                    K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
}

static int pmw3610_control_settings_set(const char *name, size_t len,
                                        settings_read_cb read_cb,
                                        void *cb_arg) {
  const char *next;
  if (!settings_name_steq(name, "control", &next) || next) {
    return -ENOENT;
  }
  if (len != sizeof(uint8_t)) {
    return -EINVAL;
  }

  uint8_t state;
  int err = read_cb(cb_arg, &state, sizeof(state));
  if (err < 0) {
    return err;
  }
  if (err != sizeof(state)) {
    return -EINVAL;
  }

  atomic_set(&control_state,
             state & (PMW3610_CONTROL_STATE_INERTIA |
                      PMW3610_CONTROL_STATE_VERTICAL_DIRECTION |
                      PMW3610_CONTROL_STATE_HORIZONTAL_DIRECTION));
  pmw3610_control_apply_to_sensors();
  pmw3610_control_mark_split_dirty();
  return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(pmw3610_control, "pmw3610", NULL,
                               pmw3610_control_settings_set, NULL, NULL);
#else
static void pmw3610_control_schedule_save(void) {}
#endif

static bool pmw3610_control_set_state(uint32_t new_state) {
  uint32_t valid_mask = PMW3610_CONTROL_STATE_INERTIA |
                        PMW3610_CONTROL_STATE_VERTICAL_DIRECTION |
                        PMW3610_CONTROL_STATE_HORIZONTAL_DIRECTION;
  uint32_t old_state =
      (uint32_t)atomic_set(&control_state, new_state & valid_mask);

  if (old_state == (new_state & valid_mask)) {
    return false;
  }

  pmw3610_control_apply_to_sensors();
  pmw3610_control_schedule_save();
  pmw3610_control_mark_split_dirty();
  return true;
}

int pmw3610_control_convert_toggle(enum pmw3610_control_kind kind,
                                   int32_t *command) {
  if (!command) {
    return -EINVAL;
  }
  if (*command == PMW3610_CONTROL_TOGGLE) {
    *command = pmw3610_control_get(kind) ? PMW3610_CONTROL_OFF
                                        : PMW3610_CONTROL_ON;
  }
  return 0;
}

int pmw3610_control_apply(enum pmw3610_control_kind kind, int32_t command,
                          int32_t value) {
  uint32_t bit = pmw3610_control_bit(kind);
  uint32_t state = (uint32_t)atomic_get(&control_state);

  switch (command) {
  case PMW3610_CONTROL_TOGGLE:
    pmw3610_control_set_state(state ^ bit);
    return 0;
  case PMW3610_CONTROL_OFF:
    pmw3610_control_set_state(state & ~bit);
    return 0;
  case PMW3610_CONTROL_ON:
    pmw3610_control_set_state(state | bit);
    return 0;
  case PMW3610_CONTROL_SYNC_STATE:
    if (kind != PMW3610_CONTROL_INERTIA) {
      return -ENOTSUP;
    }
    pmw3610_control_set_state((uint32_t)value);
    return 0;
  case PMW3610_CONTROL_SYNC_LAYERS:
    if (kind != PMW3610_CONTROL_INERTIA) {
      return -ENOTSUP;
    }
    atomic_set(&remote_layer_mask, (uint32_t)value);
    return 0;
  default:
    return -ENOTSUP;
  }
}

bool pmw3610_control_remote_layer_active(uint8_t layer) {
  if (layer >= 32) {
    return false;
  }
  return atomic_get(&remote_layer_mask) & BIT(layer);
}

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int pmw3610_control_layer_listener(const zmk_event_t *eh) {
  const struct zmk_layer_state_changed *event =
      as_zmk_layer_state_changed(eh);
  if (!event) {
    return ZMK_EV_EVENT_BUBBLE;
  }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
  pmw3610_control_mark_split_dirty();
#else
  if (event->state) {
    atomic_or(&remote_layer_mask, BIT(event->layer));
  } else {
    atomic_and(&remote_layer_mask, ~BIT(event->layer));
  }
#endif

  return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(pmw3610_control_layer, pmw3610_control_layer_listener);
ZMK_SUBSCRIPTION(pmw3610_control_layer, zmk_layer_state_changed);
#endif

static int pmw3610_control_init(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
  k_work_init_delayable(&settings_save_work, pmw3610_control_save_work);
#endif
#if IS_ENABLED(CONFIG_ZMK_SPLIT) &&                                           \
    IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) &&                             \
    PMW3610_HAS_INERTIA_BEHAVIOR
  k_work_init_delayable(&split_sync_work,
                        pmw3610_control_split_sync_work);
  pmw3610_control_schedule_split_sync(K_SECONDS(2));
#endif
  pmw3610_control_apply_to_sensors();
  return 0;
}

SYS_INIT(pmw3610_control_init, APPLICATION,
         CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#else

bool pmw3610_control_get(enum pmw3610_control_kind kind) {
  ARG_UNUSED(kind);
  return false;
}

int pmw3610_control_convert_toggle(enum pmw3610_control_kind kind,
                                   int32_t *command) {
  ARG_UNUSED(kind);
  ARG_UNUSED(command);
  return -ENOTSUP;
}

int pmw3610_control_apply(enum pmw3610_control_kind kind, int32_t command,
                          int32_t value) {
  ARG_UNUSED(kind);
  ARG_UNUSED(command);
  ARG_UNUSED(value);
  return -ENOTSUP;
}

bool pmw3610_control_remote_layer_active(uint8_t layer) {
  ARG_UNUSED(layer);
  return false;
}

#endif
