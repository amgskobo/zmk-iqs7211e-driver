Runtime regression tests
========================

Run `bash tests/run-host-docker.sh` for the CI-equivalent filter and runtime
tests, or `python3 tests/runtime/run.py` with a native C compiler available as
`cc` for this suite alone.
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

A second harness, `driver_harness.c`, lifts every other function of the
driver and compiles it with the driver's own `iqs7211e.h`, which the Zephyr
stand-ins in `zephyr/` and `iqs_stubs.h` make buildable on the host. The I2C
bus there is a word-addressed model of the sensor's memory map with fault
injection; GPIO, the work queue and input record what the driver asks of
them. It covers every step of the setup state machine and each of its
failures, the RDY interrupt, the level recheck and its back-off, touch
verification, click queueing, both suspend paths and resume.

The gcov report attributes the lifted functions to their original driver
lines, separately from the harnesses. CI requires 100% line and branch
coverage of every function in `iqs7211e.c`.

The named PM regressions in the driver harness retain completed sensor setup on
normal resume, restart silent-sensor recovery for interrupted setup, recover RDY
after the first suspend mask fails, and reject PM from the driver's own queue
before any side effect. See `tests/README.md` for the separate complete-source
real-kernel cancellation cases; host stubs alone do not prove that boundary.

`runtime-rearm` also verifies recovery after GPIO re-arm failure during a
normal read and during restoration after rejected suspend.
