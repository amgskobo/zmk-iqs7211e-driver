Runtime regression tests
========================

Run `bash tests/run-host-docker.sh` from the host for the canonical host tests,
including retirement/admission controls, filter cases and both runtime harnesses.
Use `ZMK_TEST_MODE=full` with that wrapper for retained optimization, coverage and
ABI variants. For this suite alone, run `python3 tests/runtime/run.py` from the
owner root inside a prepared Docker container with a native C compiler available
as `cc`. That focused entry does not run the other host stages or the separate
kernel event fixture described in [the test guide](../README.md).
The runner extracts the actual cleanup, report and click-worker
functions from `src/iqs7211e.c` and compiles them with fault-injecting input and
sensor stubs. It links the real coordinate filter implementation.

Covered sequences:

- Sensor reset after a click press; failed release is retried without replaying
  the remaining double/triple-click presses.
- A triple tap queues six edges and plays three ordered, synced press/release
  pairs. Each edge but the last requests the next one a click-edge delay later;
  a failed press or release is retried alone, and a failed reschedule abandons
  the remaining clicks without stranding a press.
- Repeated touch-release failures, coordinate-sync failure, and eventual recovery.
- Configured tap gestures are always emitted; routing and selective suppression
  are downstream input-processor responsibilities.
- A new contact waits for the previous contact's pending release.

The stubs do not simulate Zephyr thread scheduling, the physical sensor gesture
classifier, USB/BLE delivery, or downstream processor routing. Firmware builds
and physical-device checks complement these tests.

A second harness, `driver_harness.c`, lifts every other function of the
driver and compiles it with the driver's own `iqs7211e.h`, which the Zephyr
stand-ins in `zephyr/` and `iqs_stubs.h` make buildable on the host. The I2C
bus there is a word-addressed model of the sensor's memory map with fault
injection; GPIO, the work queue and input record what the driver asks of
them. It covers every step of the setup state machine and each of its
failures, the RDY interrupt, the level recheck and its back-off, touch
verification, click queueing, both suspend paths and resume.

The gcov report attributes the lifted functions to their original driver
lines, separately from the harnesses. Full mode requires 100% line and branch
coverage of every function in `iqs7211e.c`; quick CI runs the behavioral scenarios
with sanitizers and does not produce that coverage measurement.

The named PM regressions in the driver harness retain completed sensor setup on
normal resume, restart silent-sensor recovery for interrupted setup, recover RDY
after the first suspend mask fails, and reject PM from the driver's own queue
before any side effect. See `tests/README.md` for the separate complete-source
real-kernel cancellation cases; host stubs alone do not prove that boundary.

`runtime-rearm` also verifies recovery after GPIO re-arm failure during a
normal read and during restoration after rejected suspend.
