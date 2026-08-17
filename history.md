# Development History

## 2026-08-17

### Add Dev-v0.4_inertial-scroll branch for ZMK v0.4 (Zephyr 4.1)

- Created `Dev-v0.4_inertial-scroll` based on `main` (Zephyr 4.1 compatible with `pixart,pmw3610-alt` and `CONFIG_PMW3610_ALT`).
- Ported inertial scrolling with gesture velocity estimation, flick attack/decay, duration limit, and fade-out.
- Ported low-speed micro-motion stabilizer for smooth, drift-free pointer control.
- Ported runtime behaviors and split layer/state sync:
  - `pmw3610_inertia_toggle`
  - `pmw3610_scroll_direction_toggle`
  - `pmw3610_horizontal_scroll_direction_toggle`
- Added backward-compatible DTS bindings supporting both `pixart,pmw3610-alt` and `pixart,pmw3610`.
- Integrated Kconfig fallback macros to seamlessly support `CONFIG_PMW3610_ALT` and `CONFIG_PMW3610`.

## 2026-07-31

### Harden SPI timing, drift filtering, and pending motion

- Added a CS hold delay and the PMW3610-required delay between write commands.
- Reworked reads to hold CS across the address/data phases and wait for tSRAD.
- Added `motion-threshold` support to the ALT binding and driver.
- Dropped previously accumulated motion when a corrupt sample is detected.
- Filtered micro-motion now flushes only fresh pending motion and drops stale data.
- This affects PMW3610 SPI and input handling on both central and peripheral
  builds. CPI, layers, transforms, BLE limits, stacks, and ACL buffers are
  unchanged.

### Bound accumulated motion and isolate split-axis frames

- Added `max-report-delta` to bound one emitted report after interval-based
  motion accumulation. Excess motion is discarded instead of being released as
  delayed reports.
- Split peripherals now synchronize each non-zero axis independently. Losing
  one BLE notification can no longer leave an unsynchronized axis value to be
  combined with a later frame.
- Changed the default PMW3610 work queue priority from 1 to 10 so the input
  consumer can drain queued events during sustained sensor activity.
- Central and non-split builds still combine X/Y into one synchronized frame,
  preserving the existing HID report rate for diagonal pointer movement.
