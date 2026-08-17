# PMW3610 Driver for ZMK — Dev-v0.4_inertial-scroll

English | [日本語](README_JA.md)

## Credits & Respect

This module is based on [badjeff/zmk-pmw3610-driver](https://github.com/badjeff/zmk-pmw3610-driver).

badjeff built upon [ufan's zmk pixart sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad), [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver), and the [Zephyr PMW3610 driver](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/input/input_pmw3610.c) to create a well-structured PMW3610 driver for ZMK — with split peripheral support, per-sensor DTS configuration, and shared SPI bus compatibility. Deep respect and gratitude to badjeff and all original contributors.

This branch brings full **ZMK v0.4 (Zephyr 4.1)** support along with driver-side inertial scrolling, low-speed micro-motion stabilization, and runtime toggle behaviors.

---

## Key Features in `Dev-v0.4_inertial-scroll`

### 🟢 ZMK v0.4 (Zephyr 4.1) Full Compatibility
- Compatible string: `pixart,pmw3610-alt` (prevents conflicts with Zephyr upstream driver; `pixart,pmw3610` also supported).
- Kconfig prefix: `CONFIG_PMW3610_ALT_*` (with fallback to `CONFIG_PMW3610_*`).
- Integrated with Zephyr 4.1 input subsystem and driver APIs.

### 🟢 Driver-Side Inertial Scrolling
- **Smooth Inertia**: Emits natural, exponential-decay scrolling when trackball is flicked on designated scroll layers.
- **Time-Normalized Gesture Velocity**: Velocity is estimated over time intervals to avoid sudden runaway velocity when waking from REST mode.
- **Fade & Duration Bounds**: Configurable maximum duration (default 1800ms) with a linear fade-out (250ms).

### 🟢 Low-Speed Stabilizer (`low-speed-stabilizer`)
- Stabilizes slow, precise pointer movement by filtering micro-jitter while preserving true intentional movement. Automatically bypassed on scroll layers.

### 🟢 Runtime Control Behaviors & Split Sync
- Zero-parameter behaviors to toggle features from keymap:
  - `&pmw3610_inertia_toggle`
  - `&pmw3610_scroll_direction_toggle`
  - `&pmw3610_horizontal_scroll_direction_toggle`
- Synchronizes layer and toggle state across split Central and Peripheral halves.

---

## Installation

### 1. Add to `config/west.yml`

```yaml
manifest:
  remotes:
    - name: razilyis
      url-base: https://github.com/razilyis
  projects:
    - name: zmk-pmw3610-driver
      remote: razilyis
      revision: Dev-v0.4_inertial-scroll
  self:
    path: config
```

### 2. Configure Device Tree (`<shield>.overlay`)

```dts
#include <zephyr/dt-bindings/input/input-event-codes.h>

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 9 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,pmw3610-alt";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 2 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        cpi = <400>;
        evt-type = <INPUT_EV_REL>;
        x-input-code = <INPUT_REL_X>;
        y-input-code = <INPUT_REL_Y>;

        /* Inertial scrolling */
        inertial-scroll;
        inertial-scroll-layers = <6 7>;
        inertial-scroll-gain-pct = <130>;
        inertial-scroll-decay-pct = <99>;

        /* Low speed micro-motion stabilizer */
        low-speed-stabilizer;

        /* Power management */
        force-awake;
    };
};
```

### 3. Shield Configuration (`<shield>.conf`)

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_PMW3610_ALT=y
CONFIG_PMW3610_ALT_SMART_ALGORITHM=y
```

---

## Behaviors & Keymap Editor Compatibility

To use the behaviors in **Keymap Editor** (nickcoutsos/keymap-editor), define them in `behaviors { ... }` in your `.keymap` file:

```dts
/ {
    behaviors {
        pmw3610_inertia_toggle: pmw3610_inertia_toggle {
            compatible = "zmk,behavior-pmw3610-inertia-toggle";
            #binding-cells = <0>;
            label = "PMW3610_INERTIA_TOGGLE";
            display-name = "PMW3610 Inertia Toggle";
        };

        pmw3610_scroll_direction_toggle: pmw3610_scroll_direction_toggle {
            compatible = "zmk,behavior-pmw3610-scroll-direction-toggle";
            #binding-cells = <0>;
            label = "PMW3610_SCROLL_DIRECTION_TOGGLE";
            display-name = "PMW3610 Scroll Direction Toggle";
        };

        pmw3610_horizontal_scroll_direction_toggle: pmw3610_horizontal_scroll_direction_toggle {
            compatible = "zmk,behavior-pmw3610-horizontal-scroll-direction-toggle";
            #binding-cells = <0>;
            label = "PMW3610_HORIZONTAL_SCROLL_DIRECTION_TOGGLE";
            display-name = "PMW3610 Horizontal Scroll Direction Toggle";
        };
    };
};
```
