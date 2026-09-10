Runtime regression tests
========================

Run `python3 tests/runtime/run.py` with a native C compiler available as `cc`.
The runner extracts the actual cleanup, report, click-worker and layer-listener
functions from `src/iqs7211e.c` and compiles them with fault-injecting input and
sensor stubs. It links the real coordinate filter implementation.

Covered sequences:

- Sensor reset after a click press; failed release is retried without replaying
  the remaining double/triple-click presses.
- Repeated touch-release failures, coordinate-sync failure, and eventual recovery.
- Cleanup only deactivates the driver-owned slider once.
- Manual `scroll-layers` and the driver-owned `scroll-slider-layer` contacts
  followed by layer deactivation and a delayed tap; the two paths use the same
  tap-suppression history.
- Brief manual or slider layer activation between sensor reports.
- ZMK Studio layer reordering: a configured trigger layer ID is still matched
  after its visual layer index changes.
- Normal taps after a suppressed contact, including when no delayed tap arrives.
- A new contact waits for the previous contact's pending release.

The stubs do not simulate Zephyr thread scheduling, the physical sensor gesture
classifier, USB/BLE delivery, or downstream processor routing. Firmware builds
and physical-device checks complement these tests.
