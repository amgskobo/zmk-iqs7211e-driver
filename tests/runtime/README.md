Runtime regression tests
========================

Run `python3 tests/runtime/run.py` with a native C compiler available as `cc`.
The runner extracts the actual cleanup, report and click-worker
functions from `src/iqs7211e.c` and compiles them with fault-injecting input and
sensor stubs. It links the real coordinate filter implementation.

Covered sequences:

- Sensor reset after a click press; failed release is retried without replaying
  the remaining double/triple-click presses.
- Repeated touch-release failures, coordinate-sync failure, and eventual recovery.
- Configured tap gestures are always emitted; routing and selective suppression
  are downstream input-processor responsibilities.
- A new contact waits for the previous contact's pending release.

The stubs do not simulate Zephyr thread scheduling, the physical sensor gesture
classifier, USB/BLE delivery, or downstream processor routing. Firmware builds
and physical-device checks complement these tests.
