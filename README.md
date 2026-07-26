# zmk-pmw3610-driver — Dev-v0.3_inertial-scroll

English | [日本語](README_JA.md)

## Credits and Respect

This module is based on [badjeff/zmk-pmw3610-driver](https://github.com/badjeff/zmk-pmw3610-driver).

badjeff built upon [ufan's ZMK PixArt sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad) and [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver) to create a well-structured PMW3610 driver for ZMK v0.3, with split peripheral support, per-sensor devicetree configuration, and shared SPI bus compatibility. That work laid the foundation for trackball integration in ZMK, and this branch would not exist without it. Deep respect and gratitude to badjeff for these contributions to the community.

This branch adds the improvements described below.

---

## Dev-v0.3_inertial-scroll overview

### Stability and power management

- **Level-triggered interrupt:** Uses `GPIO_INT_LEVEL_ACTIVE` instead of an edge trigger to avoid losing motion events while the interrupt is disabled.
- **Fail-safe initialization:** Retries a failed SPI initialization up to three times. After the retry burst is exhausted, it restarts initialization after a one-second backoff by default. Configure the delay with `CONFIG_PMW3610_INIT_RETRY_BACKOFF_MS`.
- **FAULT recovery and jump prevention:** Discards accumulated movement (`dx`, `dy`) and inertia state after a FAULT so that recovery cannot produce a pointer jump.
- **Input queue protection:** Keeps diagonal X/Y movement in one synchronized report and waits for the matching Y event after X is queued, preventing incomplete events from leaking into a later report.
- **Serialized inertia state:** Protects inertia work, toggles, and FAULT recovery from recreating stale inertia after it has been stopped.
- **IDLE power fix:** Prevents `force-awake-4ms-mode` from retaining the 4 ms rate after entering IDLE. The sensor returns to its normal 8 ms default rate in IDLE to save power.

### Driver-side inertial scrolling

On layers where the PMW3610 is used as a scroll device, scrolling can continue with decaying momentum after the user releases the ball.

---

## Installation

### 1. Add the module to west.yml

Add the following entries to `config/west.yml`:

```yaml
manifest:
  remotes:
    - name: razilyis
      url-base: https://github.com/razilyis
  projects:
    - name: zmk-pmw3610-driver
      remote: razilyis
      revision: Dev-v0.3_inertial-scroll
  self:
    path: config
```

### 2. Configure the board overlay

Add the sensor configuration to `<board>.overlay`. Change the pin assignments to match your hardware:

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                    <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };
    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                    <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

#include <zephyr/dt-bindings/input/input-event-codes.h>

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,pmw3610";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        cpi = <600>;
        evt-type = <INPUT_EV_REL>;
        x-input-code = <INPUT_REL_X>;
        y-input-code = <INPUT_REL_Y>;

        /* Drift filter. Set to 0 to disable. */
        motion-threshold = <1>;

        /* Reject abnormal single-sample movement. Optional; default 512. */
        max-motion-delta = <512>;

        /* Inertial scrolling (optional). */
        inertial-scroll;
        inertial-scroll-layers = <6 7>; /* Enabled layers. Omit to enable on all layers. */
        inertial-scroll-gain-pct = <130>;
        inertial-scroll-decay-pct = <99>;
        inertial-scroll-interval-ms = <10>;
        inertial-scroll-threshold = <4>;

        /* Power control (optional). */
        force-awake;          /* Keep the sensor awake while ZMK is ACTIVE. */
        force-awake-4ms-mode; /* Force 4 ms sampling while ACTIVE for 250 Hz over USB. */

        // swap-xy;   /* Optional: swap the X and Y axes. */
        // invert-x;  /* Optional: invert the X axis. */
        // invert-y;  /* Optional: invert the Y axis. */
    };
};

/ {
    trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
    };
};
```

### 3. Configure the shield

Add the following options to `<shield>.conf`:

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_PMW3610=y
# CONFIG_PMW3610_REPORT_INTERVAL_MIN=12  # Optional minimum report interval (ms)
# CONFIG_PMW3610_LOG_LEVEL_DBG=y         # Optional debug logging
# CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS=300  # See troubleshooting notes
```

---

## Devicetree properties

### Basic configuration

| Property | Type | Default | Description |
|---|---|---|---|
| `irq-gpios` | phandle-array | required | Motion interrupt GPIO |
| `cpi` | int | 600 | Counts per inch, from 200 to 3200 in steps of 200 |
| `evt-type` | int | required | Input event type, such as `INPUT_EV_REL` |
| `x-input-code` | int | required | X-axis input code |
| `y-input-code` | int | required | Y-axis input code |
| `motion-threshold` | int | 1 | Discard a sample when the absolute values of both X and Y are at or below this drift-filter threshold. Set to `0` to disable |
| `max-motion-delta` | int | 512 | Discard a single sample when the absolute value of X or Y is at or above this value, preventing abnormal pointer jumps and inertia generation. Valid range: 1 to 2048 |
| `swap-xy` | boolean | — | Swap the X and Y axes |
| `invert-x` | boolean | — | Invert the X axis |
| `invert-y` | boolean | — | Invert the Y axis |

### Power control

| Property | Type | Description |
|---|---|---|
| `force-awake` | boolean | Keep the sensor awake while ZMK is ACTIVE. Normal downshifting resumes in IDLE or SLEEP |
| `force-awake-4ms-mode` | boolean | Force 4 ms sampling while `force-awake` is active. Intended for high-rate, direct USB use |

### Inertial scrolling

| Property | Type | Default | Description |
|---|---|---|---|
| `inertial-scroll` | boolean | — | Enable inertial scrolling |
| `inertial-scroll-gain-pct` | int | 130 | Gain used to derive initial inertial velocity from the smoothed gesture velocity, in percent |
| `inertial-scroll-decay-pct` | int | 99 | Velocity retained on each inertial tick, in percent. Smaller values stop sooner |
| `inertial-scroll-decay-basis-points` | int | 0 | Optional high-precision retention in 0.01% units. For example, `9920` means 99.20%. A value of `0` uses `inertial-scroll-decay-pct` |
| `inertial-scroll-interval-ms` | int | 10 | Interval between synthetic inertial scroll reports, in milliseconds |
| `inertial-scroll-threshold` | int | 4 | Fixed-point Q8 velocity threshold below which inertial scrolling stops |
| `inertial-scroll-max-velocity` | int | 32 | Maximum initial inertial velocity in whole scroll steps per tick |
| `inertial-scroll-max-duration-ms` | int | 1800 | Maximum duration of one inertial tail, in milliseconds |
| `inertial-scroll-fade-duration-ms` | int | 250 | Linear fade duration before the maximum inertial duration is reached. Set to `0` to disable |
| `inertial-scroll-layers` | array | — | ZMK layer numbers where inertial scrolling is allowed. Omit to allow all layers |
| `scroll-direction-toggle` | boolean | — | Allow direction-toggle behaviors to affect a scroll-only sensor that does not enable `inertial-scroll` |
| `vertical-scroll-uses-x-axis` | boolean | false | Treat raw X as vertical and raw Y as horizontal for direction toggling on a sensor mounted with a 90-degree rotation |

Use `inertial-scroll-layers` when the same PMW3610 acts as both a pointer and a scroll device. List only the scroll layers:

```dts
inertial-scroll-layers = <6 7>;
```

Initial inertial velocity is derived from recent gesture velocity normalized by the actual report interval, rather than from only the final report. For the first report of a new gesture, the driver estimates the sample period from the RUN period and report-throttling configuration while ACTIVE, or from the estimated RUN/REST timing after IDLE. This preserves short flicks without creating excessive inertia from accumulated movement after waking from REST.

The filter follows acceleration quickly and deceleration gradually, preserving momentum when the user naturally slows at the end of a fast flick. An input gap longer than 80 ms or a direction reversal resets velocity history and begins a new gesture.

For a sensor mounted at 90 degrees where raw X becomes vertical scroll, add `vertical-scroll-uses-x-axis;` to the sensor node.

### Low-speed pointer stabilization

| Property | Type | Default | Description |
|---|---|---|---|
| `low-speed-stabilizer` | boolean | — | Enable low-speed micro-motion stabilization |
| `low-speed-stabilizer-threshold` | int | 1 | Largest absolute delta treated as micro motion |
| `low-speed-stabilizer-timeout-ms` | int | 30 | Idle time before direction history is reset. The first micro movement after stopping is emitted immediately |

Repeated small movements in the same direction are emitted without losing distance. An isolated reversal is held for confirmation. If the following movement returns to the original direction, the reversal is treated as noise and cancelled; if the reversal continues, it is emitted as an intentional direction change. Stabilization is bypassed automatically on layers listed in `inertial-scroll-layers`.

### Safety and recovery Kconfig options

| Option | Default | Description |
|---|---|---|
| `CONFIG_PMW3610_INPUT_REPORT_TIMEOUT_MS` | 5 | Maximum time to wait for each event to enter the input queue, from 1 to 20 ms. Prevents a stalled input consumer from blocking the system work queue |
| `CONFIG_PMW3610_INIT_RETRY_BACKOFF_MS` | 1000 | Delay before restarting full initialization after three failed retries, from 100 to 10000 ms |

---

## Inertial scroll toggle behavior

The keymap can expose a zero-parameter behavior that toggles inertial scrolling.

Include the behavior in the `.keymap` file:

```dts
#include <behaviors/pmw3610_inertia_toggle.dtsi>
```

Assign it to any key:

```dts
&pmw3610_inertia_toggle
```

## Vertical scroll direction toggle behavior

This zero-parameter behavior toggles the vertical scroll direction:

```dts
#include <behaviors/pmw3610_scroll_direction_toggle.dtsi>
```

Assign it to any key:

```dts
&pmw3610_scroll_direction_toggle
```

In a split configuration, this is a Global Behavior delivered to both Central and Peripheral. The Central resolves the new ON/OFF state and sends that explicit state instead of sending a relative toggle command.

On sensors with `inertial-scroll-layers`, only the listed layers are affected, so normal pointer direction does not change. Direction toggles affect sensors with `inertial-scroll`; pointer-only sensors are not affected. Add `scroll-direction-toggle;` only when a scroll-only sensor without inertia must also respond. Changing direction stops any active inertial tail.

## Horizontal scroll direction toggle behavior

This zero-parameter behavior toggles the horizontal scroll direction:

```dts
#include <behaviors/pmw3610_horizontal_scroll_direction_toggle.dtsi>
```

Assign it to any key:

```dts
&pmw3610_horizontal_scroll_direction_toggle
```

Like the vertical behavior, it is delivered globally and affects both direct scrolling and inertial scrolling on eligible sensors. With `vertical-scroll-uses-x-axis`, X is vertical and Y is horizontal. Without it, Y is vertical and X is horizontal.

## Initial control behavior state

The initial state when no saved settings exist is:

| Control | Initial state |
|---|---|
| Inertial scrolling | ON |
| Vertical direction inversion | OFF |
| Horizontal direction inversion | OFF |

When `CONFIG_SETTINGS=y` and a saved state exists, the driver restores that state during startup.

## Single-sided sensor configurations

The module supports zero, one, or multiple PMW3610 devices. Control behavior code is built independently of `CONFIG_PMW3610`, allowing sensors on only the Central, only a Peripheral, or both halves. A half without a sensor participates only in state synchronization.

A split Peripheral cannot read the Central keymap layer directly, so active layers are synchronized through the control behavior. This allows `inertial-scroll-layers` to work for a sensor located only on a Peripheral. Synchronization happens at startup and after layer or toggle changes, with a finite retry count. There is no permanent five-second polling loop; after retries are exhausted, a later reconnection is synchronized on the next layer change or toggle action.

Layer synchronization uses the `pmw3610_inertia_toggle` behavior as its transport path. When using `inertial-scroll-layers` on a split Peripheral sensor, include `pmw3610_inertia_toggle.dtsi` and reference `&pmw3610_inertia_toggle` from the keymap so the behavior node remains enabled. Each provided behavior node uses `/omit-if-no-ref/`, so an included but unreferenced node is removed during the build.

### Using keymap-editor

[nickcoutsos/keymap-editor](https://github.com/nickcoutsos/keymap-editor) cannot add behaviors from an external west module through its UI. Existing bindings already present in the `.keymap` file are preserved.

As a workaround, define the behavior node in the config repository's `.keymap` or manually assign:

- `&pmw3610_inertia_toggle`
- `&pmw3610_scroll_direction_toggle`
- `&pmw3610_horizontal_scroll_direction_toggle`

The editor preserves these existing bindings even though it does not parse the external west module directly.
