# PMW3610 driver implementation for ZMK

#### ⚠️ Notice: `dev-v0.3_against-runaway` branch differences vs `zmk-0.3`
This branch (`dev-v0.3_against-runaway`) incorporates several critical stability and power-management fixes to prevent cursor runaway and device freezing issues present in the base `zmk-0.3` branch. Key improvements include:
- **Robust Interrupt Handling**: Reverted to level-triggered interrupts (`GPIO_INT_LEVEL_ACTIVE`) to prevent missing motion events during interrupt disable periods.
- **Fail-Safe Initialization (Anti-Bricking)**: Added a retry mechanism (up to 3 times) for SPI initialization. If it fails consecutively, it backs off for 10 seconds before restarting the async flow, preventing battery drain from infinite loops and permanent sensor bricking.
- **Fault Recovery & Anti-Jump**: Unifies fault detection and explicitly clears accumulated motion (`dx`, `dy`) upon recovery to prevent unexpected cursor jumps (runaways) after waking up from a fault state.
- **Proper IDLE Power Saving**: Fixed a bug where `force_awake_4ms_mode` would incorrectly keep the sensor at a 4ms rate even when ZMK enters the IDLE state. It now properly drops to the 8ms default rate during IDLE, saving battery life.

#### `Dev-v0.3_inertial-scroll-v2` inertial scrolling
This branch keeps the runaway/fault recovery fixes above and adds driver-side inertial scrolling for PMW3610 scroll layers.

Available DTS properties:
- `inertial-scroll`
- `inertial-scroll-gain-pct`
- `inertial-scroll-decay-pct`
- `inertial-scroll-interval-ms`
- `inertial-scroll-threshold`
- `inertial-scroll-layers`

Use `inertial-scroll-layers` when the same PMW3610 is a pointer on one layer and a scroll source on another. Specify layer numbers directly, for example `inertial-scroll-layers = <6 7>;`. Omit it when inertial scrolling should be allowed on all layers.

---

This work is based on [ufan's zmk pixart sensor drivers](https://github.com/ufan/zmk/tree/support-trackpad), [inorichi's zmk-pmw3610-driver](https://github.com/inorichi/zmk-pmw3610-driver), and [Zephyr PMW3610 driver](https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/input/input_pmw3610.c).

This driver had been tested on [my PMW3610 breakout board](https://github.com/badjeff/pmw3610-pcb).


#### What is different to [inorichi's driver](https://github.com/inorichi/zmk-pmw3610-driver)
- Compatible to be used on split peripheral shield.
- Replaced `CONFIG_PMW3610_ORIENTATION_*` with ~~`CONFIG_PMW3610_SWAP_XY` and `PMW3610_INVERT_*`~~ device tree node attributes `swap-xy;`, `invert-x;` and `invert-y;`. Then now, it can used on [leylabella](https://github.com/badjeff/leylabella), which has different sensor breakout pcb orientation on one device.
- Moved `CONFIG_PMW3610_CPI` to device tree node `.dts/.overlay`. It is now allowed to setup diffeent config for multi-sensor on single shield. In case of building typical mouse shield, we use one movment sensor on bottom, and another sensor for scrolling on top. Those settings could be distinguishable.
- Features for scroll-mode, snipe-mode, and auto-layer are no longer needed to be provided from sensor driver. Those settings is now configurable in keymap with layer-based `zmk,input-listener`, instead of setup static value in shield config files.
- Seperating sampling rate and reporting rate. It reports accumulated XY axes displacement between data ready interrupts. You will still feeling lag and jumpy in noisy radio hell, but the cursor traction should being lossless, and predicable in exact terms.
- Default to use power saving config. Applying shorter-than-default downshift time to PMW3610.
- Deprecated manual *chip-select*. Refactored to use Zephyr's `spi_transceive_dt()`. That allow the sensor could be attacted to a shared SPI bus, works along with others SPI peripherals, such as display module.

## Installation

Include this project on ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  remotes:
    ...
    # START #####
    - name: razilyis
      url-base: https://github.com/razilyis
    # END #######
    ...
  projects:
    ...
    # START #####
    - name: zmk-pmw3610-driver
      remote: razilyis
      revision: Dev-v0.3_inertial-scroll-v2
    # END #######
    ...
  self:
    path: config
```

Update `board.overlay` adding the necessary bits (update the pins for your board accordingly):

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
        motion-threshold = <1>; /* optional: set 0 to disable drift filtering */
        inertial-scroll; /* optional: enable driver-side scroll inertia */
        inertial-scroll-layers = <6 7>; /* optional: only active on these layers */
        inertial-scroll-gain-pct = <130>;
        inertial-scroll-decay-pct = <99>;
        inertial-scroll-interval-ms = <10>;
        inertial-scroll-threshold = <4>;
        // swap-xy; /* optional */
        // invert-x; /* optional */
        // invert-y; /* optional */
        evt-type = <INPUT_EV_REL>;
        x-input-code = <INPUT_REL_X>;
        y-input-code = <INPUT_REL_Y>;

        force-awake;
        /* keep the sensor awake while ZMK activity state is ACTIVE,
           fallback to normal downshift mode after ZMK goes into IDLE / SLEEP mode.
           thus, the sensor would be a `wakeup-source` */

        force-awake-4ms-mode;
        /* while force-awake is acitvated, enable this mode to force sampling per 
           4ms, where the default sampling rate is 8ms. */
        /* NOTE: apply this mode if you need 250Hz with direct USB connection. */
    };
};

/ {
  trackball_listener {
    compatible = "zmk,input-listener";
    device = <&trackball>;
  };
};
```

Enable the driver config in `<shield>.config` file (read the Kconfig file to find out all possible options):

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_POINTING=y
CONFIG_PMW3610=y
# CONFIG_PMW3610_SWAP_XY=y // <-- deprecated, use swap-xy; instead
# CONFIG_PMW3610_INVERT_X=y // <-- deprecated, use invert-x; instead
# CONFIG_PMW3610_INVERT_Y=y // <-- deprecated, use invert-y; instead
# CONFIG_PMW3610_REPORT_INTERVAL_MIN=12
# CONFIG_PMW3610_LOG_LEVEL_DBG=y
# CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS=300 // <--see Troubleshooting
```

## Troubleshooting

If you are getting `Incorrect product id 0xFF (expecting 0x3E)!` on `nice_nano_v2` board from the log, you'd want to apply `CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS=1000` in your shield .conf/.overlay file. Due to this driver doesn't offer module dependancy setting, that would ensure external power (to enable VCC pin on board) is ready, the `CONFIG_PMW3610_INIT_POWER_UP_EXTRA_DELAY_MS` would use to add extra one second delay of power up.
