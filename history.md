# Development History

## 2026-07-31

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
