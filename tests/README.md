# Module-owned tests

Run from this checkout. Docker + Bash is the host entry point; no keyboard
checkout, shared test dispatcher or simulator repository is required.

```bash
bash tests/run-host-docker.sh
ZMK_TEST_MODE=full bash tests/run-host-docker.sh
```

On PowerShell, set `$env:ZMK_TEST_MODE="full"` for the second command
and invoke Git Bash explicitly. Remove the variable for the default quick run.

Quick executes **all host behavioral fixtures and scenarios** once with
AddressSanitizer/UndefinedBehaviorSanitizer. Full repeats those same assertions
under the retained optimization, coverage and (where already present) 32-bit
variants. CI uses quick for pushes to `main` and for pull requests, and full
for scheduled/manual runs. Neither host mode invokes the separate API, tap-range,
kernel PM or kernel event entries documented below.
Unknown mode values fail; there is no silent skipped success.

Launch `tests/run-host-docker.sh` from the host. The direct Python and filter
commands below are inner entries: run them from this owner root inside a prepared
Docker container. The native and firmware entries need resolved workspaces at
the revisions in [integration/dependencies.json](integration/dependencies.json)
and the selected toolchain already installed. Mount required source inputs
read-only and retain evidence outside them on a mounted volume or export it
before removing the container. These entries do not prepare or fetch dependencies.

The canonical host entry also loads the inexpensive kernel-event runner controls
through `tests/test_kernel_event_runner.py`. Invalid evidence or selections fail
there before the separate native builds; the controls do not build firmware.

## Retained behavioral contracts

| Fixture | Assertions |
| --- | --- |
| [filter/run.sh](filter/run.sh) | Filtering boundaries and coordinate continuity |
| [runtime/driver_harness.c](runtime/driver_harness.c) | Initialization, RDY/retry, I2C failures, recovery and PM state |
| [runtime/reports_harness.c](runtime/reports_harness.c) | Contact and button report lifecycle |

Timing/FLOW/stack collectors have been retired. The existing reports and driver
fixtures retain input lifecycle, bus/recovery and PM assertions.
See [the retirement map](retired-measurements.md).

Both normal and `IQS7211E_FILTER_BYPASS` builds run the lifecycle/fault fixtures.
The filtered output traces remain unchanged; a separate bypass trace requires
immediate one-unit motion, reversal, stopping, independent contacts, rotation,
invalid-first/held coordinates and balanced absolute releases. The pure filter
suite also covers both modes, reset/independence/boundaries and an actual negative
control that rejects the normal filter using the bypass expectations. Full mode
retains each mode's line/branch coverage gates. Board acceptance remains separate.

Each fixture keeps its existing normal, boundary/invalid, independent-state,
reset/cancellation and failure/recovery scenarios where applicable. Checks
assert ordered outputs and explicit time inputs. Coverage is a full-mode gate,
not a reason to replace assertions or a hardware completeness claim.

The canonical host entry also runs `test_runtime_coverage.py` in both modes.
Recording compiler/gcov tools drive the actual full runtime entry and require
rejection of empty reports, unrelated metadata and a report measuring only one
gated function. Every gated range must contain executable lines; `lines 0/0`
cannot pass. Existing missed-line/branch rejection and real full-mode behavioral
measurement remain unchanged. The recording tools test admission, not driver
behavior or physical timing.

## Separate integration and physical contracts

The owner now has an independent API/DT firmware fixture below
`tests/integration/`. The consuming keyboard additionally builds its actual
wiring for changed APIs, DTS/Kconfig or registration. Run focused driver or physical checks
for affected bus, interrupt and power boundaries. Do not run all firmware
variants for a pure arithmetic edit. Host extracted functions and controlled
work queues do not prove real scheduling or transport delivery.

## Current test ownership and compatibility

Run the relevant public interface/build checks on both unchanged stock ZMK
and DYA when this owner supports both. See the selected core in the manifest.

Canonical entries (run from the repository root):

```bash
bash tests/run-host-docker.sh
```

[maintenance.json](maintenance.json) records supported cores, current commands
and explicit gaps. It is a reference map, not a passing test or a claim that
registration/coverage totals establish completeness. Preserve ordered outputs,
raw timestamps and the first failure. Read actual dependencies before execution.

## Supported-core interface fixture

```bash
python3 tests/integration/run.py --workspace /absolute/resolved/workspace \
  --core stock-zmk --output /absolute/new/output
# Repeat with the separately resolved DYA workspace and --core dya.
```

The runner verifies clean pinned core/Zephyr identities, stages an unchanged
owner copy once, and builds only against already resolved dependencies. It
checks required Kconfig, actual compiled source units and the UF2. Outputs
include source/dependency identities, exact commands, config, DTS and binaries.
The fixture establishes API/DT compilation, not physical wiring or runtime
timing. No network update or dependency source patch is performed.

The XIAO fixture uses the production Nordic TWIM backend with 32-byte concat/flash
buffers for a register byte plus the driver's largest 30-byte settings block, and
active-low RDY. Inspect effective config/DTS and linked backend symbols: compilation
alone does not exercise those transfers, interrupts or physical timing.

## PM boundary regressions

The driver host harness additionally checks `failed-mask`, `retained-resume`,
`interrupted-resume`, `own-queue-pm` and `runtime-rearm`. All run in the default selection in
both filter modes. A focused diagnostic uses, for example:

```bash
IQS_TEST_CASE=failed-mask ZMK_TEST_MODE=quick python3 tests/runtime/run.py
```

The actual kernel check compiles the complete driver unchanged once, uses the
real private work queue and GPIO emulator, and exercises suspend from the
currently executing ordinary and delayed click work items, plus the real input
consumer with its message queue full and a button release owed. Each must return
`-EWOULDBLOCK` with state untouched; a subsequent external suspend/resume must
work, delivering the retained release exactly once. All three cases run three
times with the same binary. A premature exit 0 or
timeout is a failure unless the named completion marker and exit 0 both match.

```bash
python3 tests/kernel/run.py --workspace /resolved/stock --core stock-zmk \
  --output /fresh/kernel-stock
python3 tests/kernel/run.py --workspace /resolved/dya --core dya \
  --output /fresh/kernel-dya
python3 tests/kernel/run.py --workspace /resolved/dya --core dya \
  --output /fresh/invalid-priority --negative-control invalid-priority
```

The explicit negative control sets the queue priority to 15 with 15 preemptive
priorities (legal priorities 0..14) and requires the named compile-time rejection.
It must fail before the guard exists. These entries use the same pinned clean
core/Zephyr selection as the interface fixture, do not fetch dependencies, and
save commands, config, binary hash, source identities and raw ordered repeats.
This is a work-cancellation check, not I2C, RDY electrical timing or sensor
reseed validation. Those still require actual board evidence.

Threaded-input suspend uses the `input` thread handle exported by the pinned
Zephyr input subsystem in both supported cores; those revisions have no public
accessor for it. Both complete-source native links and supported-core firmware
builds check this dependency. A caller on that thread must defer suspend to
another thread after `-EWOULDBLOCK`, just like a caller on the private work queue.
Synchronous input mode does not reference the threaded backend's handle.

Both API/kernel entries reject relative, existing, linked and dangling result
paths before invoking west. The guard is exercised by
`python3 -m unittest discover -s tests/integration -p 'test_*.py' -v`, included
in the canonical host/CI entry. Failures are preserved in fresh output paths.
The GitHub workflow runs host tests; native/API/board checks remain separate
documented local gates and must not be inferred from its green host job.

Runtime IRQ-arm failures retain the level-recheck/retry chain, including after
a rejected suspend. A failed first suspend mask preserves pending report state
because the work has not yet been drained. Initial GPIO setup errors still fail
device initialization. The host regression injects both normal re-arm and
rejected-suspend restore failures and verifies successful retry.

## Native-width verification timeouts

The two variable verification-work reservations share the private
`src/iqs7211e_timeout.h` conversion. The full uint16 range of 0..65535 ms, zero
verification-disable behavior, ceiling rounding, private work-queue ownership,
IRQ/retry/I2C/PM and ordered report/release policy remain unchanged. Clock rates
through 65536 Hz use a checked uint32 numerator. Higher rates split seconds and
the fractional product while retaining the configured kernel tick width; no
software 64-bit divider or new clock restriction is introduced.

`test_timeout_width.py`, selected by the canonical host entry, executes the same
`timeout/test_timeout.c` at 14 clock rates against an independent wide ceiling
oracle for every uint16 wait. Full retains optimized, ASan/UBSan, coverage and
m32 execution, with both 32/64-bit tick forms. Actual header line/branch coverage
must be 100% with nonzero measured counts. The actual object must have no wide
arithmetic helpers. Four identified floor/product/fraction/seconds faults each
must compile and fail qualified C assertions three times per tick form. The
compiled wide-divider control stays numerically correct but must fail the
object-width guard. All previous filtered/bypass and fault fixtures remain.

The complete-source kernel PM fixture additionally compares the actual production
helper against pinned Zephyr conversion macros at 7 positive clock rates over
all 65536 waits. It keeps the same work/click/input cases and PM completion markers,
three repeats each on both supported cores. The fixture still models GPIO and
queue boundaries only; electrical sensor/I2C/RDY, physical latency and power
remain unmeasured. Retired collectors are not restored.

```bash
python3 -m unittest discover -s tests -p test_timeout_width.py -v
```

## Kernel event fixture

`tests/kernel/events/` runs the complete driver on the real kernel and asserts what
comes out of it. Devicetree instances are initialized by the kernel. Transactions
go through the real I2C emulator controller, RDY edges are raised from timer ISRs
on the emulated GPIO, and work items, the input queue and thread, PM device actions
and the ZMK input listener up to the HID report are the real ones. Only the sensor
is a model (`sensor_model.h`): its registers and bits are literals from the memory
map, independent of the driver's macros.

```bash
python3 tests/kernel/events/run.py --workspace /resolved/dya --core dya \
  --output /fresh/events-dya
python3 tests/kernel/events/run.py --workspace /resolved/stock --core stock-zmk \
  --output /fresh/events-stock
python3 tests/kernel/events/run.py --workspace /resolved/dya --core dya \
  --output /fresh/events-mutations --negative-control mutations
```

[kernel/events/cases.json](kernel/events/cases.json) names every case and what it
asserts: boot and setup recovery, contacts, rotation, touch verification, taps, RDY
and bus failures, a stopped input consumer, runtime resets, PM, two sensors on one
queue, and HID reports. The entry requires the fixture to list exactly those cases.
Each case runs three times on one binary and must print its completion marker, exit
0 and repeat one identical printed trace, including raw ticks in input, HID, bus and RDY
lines. The fixture builds once, at the 32768 Hz tick rate of the production boards;
the native_sim default of 1000 Hz adds no production case and is not built.
Assertions check exact report contents and order, selected bus transactions, and
bounds on observed delays. They do not assert exact requested delays. Time is
virtual; observed scheduling comes from the kernel rather than host load.
Every repeat requires exactly one complete named completion line, nonempty
input/HID/bus/RDY observations and no `IQSFAIL` line. An empty case selection,
completion text embedded in diagnostics or duplicated completions cannot pass.
`--case NAME` selects a case for diagnosis and can be repeated to select several
cases. Repeated names select a case once, in first-occurrence order; each selected
case still runs three times. Its stdout/stderr/exit files are written once per
repeat, preserving the first execution evidence.

Expected traces are derived from the driver's contract, including the filter's
exact output for a given input; none is captured from a run. The negative control
applies each fault in [kernel/events/mutations.json](kernel/events/mutations.json)
to a disposable copy, at the same tick rate. The unchanged baseline is an independent
copy of the first successful worker's pristine executable, config, DTS, CMake cache
and build evidence; its SHA-256 and `source_worker` are recorded. It runs concurrently
with `--jobs` workers (default 4, capped by the number of selected faults) and must
pass every case named by the selected faults with three identical-binary repeats.
There is no extra baseline build, and workers mutate only their own executable.
That core's baseline can also supply its normal product evidence when its
`baseline/results.json` covers exactly every name in `cases.json`, with three
successful repeats and identical nonempty traces per case, and the retained
binary/config/source/dependency identities match the selected configuration.
Check that coverage before omitting a separate product build and run. A selected
`--mutation` baseline that omits catalogue cases supplies only focused evidence;
the other supported core still needs its own complete product result. Keep the
normal baseline results separate from the faults' expected failures.
Each worker builds a pristine copy once. A worker then applies one fault at a time,
rebuilds incrementally, runs the fault's named cases once, and restores the file
before the next. Mutation matches must have the exact declared count, restricted
to the named function body where specified. A fault names one case, or a list
when one edit is the designated check of several cases; it must build and be
rejected by the `IQSFAIL` assertion of every named case. A fault that does not
build, that survives, or that never ran fails the control.
An empty expected-case list or one containing duplicate names is rejected before
any builds. Every named case must run an assertion with its own evidence files;
duplicates cannot overwrite an earlier result or inflate the case count.
The rejection must be an entire `IQSFAIL case=NAME line=NUMBER: ...` line and
exit 1. Timeouts, signals, other exit codes and failures from similarly named
cases do not qualify, even if their output mentions the expected marker.
Every worker must finish successfully too: a pristine-build failure or an exception
(including its traceback) is a terminal failure even if other workers handle every
fault. The top-level `results.json` records `baseline_passed`, the baseline identity,
result/exception, elapsed seconds, `workers` and `mutations`. If every pristine
build fails, baseline admission fails without leaving a blocked waiter. Each `worker-N/`
retains its pristine executable's SHA-256, `.config`, `zephyr.dts`, `CMakeCache.txt`
and build log/exit. Each fault row records the pre-build executable's SHA-256 as
`previous_binary_sha256`; a successful incremental build also records the resulting
`binary_sha256`. A hash equal to either the worker's pristine hash or the pre-build
hash is `STALE-BINARY`, fails the control and skips the fault's cases. A previous
fault's failing executable therefore cannot establish rejection of the next fault.
Every result other than `REJECTED` retains the available executable in the fault directory, including
the old executable if a rebuild fails; failed incremental build logs are retained.
Controls are reported apart from product results. Pristine builds use two compiler
jobs for each worker. Set `CMAKE_BUILD_PARALLEL_LEVEL=2` to bound
incremental builds, and cap the container's total CPU and memory for all workers
and the baseline together. Staging, pristine build and every subprocess have
separate timing JSON files; each fault records incremental-build, case and total
seconds. Compare like image/input/cache conditions and keep preparation separate.
Each case belongs to the catalogue only if no other case already establishes what
it checks, and each fault only if no other fault makes the same edit or the same
class of edit to the same statement.
The runner admission controls are included in canonical host/CI tests; focus them in the prepared container with:
`python3 -m unittest discover -s tests/kernel/events -p test_runner.py -v`.

The transport model checks actual Zephyr message flags: one uninterrupted
register-address fragment, followed by contiguous write data or a repeated-START
read, then STOP. `IQSBUS` retains both flags, ACK/NACK result and raw tick time.
`IQSRDY` retains every modelled level transition, its time and the driver's
IRQ-armed state immediately before that edge. These observations join the
three-repeat trace comparison. `read-missing-restart` and
`write-address-stopped` must build and fail the named shape assertion; both
survive without that check. Zephyr can add the final STOP itself, so removing
that flag before `i2c_transfer()` is not a qualifying fault at this configuration.
These are controller messages and emulated RDY, not physical START/STOP/IRQ traces.

Silent-boot recovery additionally covers an accepted reset that the model ignores:
the next command at least 200 ms later restores setup and clicks. Ignoring every
accepted reset exhausts the same three-command budget and leaves a quiet bus.
`boot-kick-success-not-retried` must fail both cases; `boot-kick-unbounded` must
fail both the absent/NACKed and present/accepted-command limits.

The sensor model follows IQS7211E datasheet v1.4 §§9.3.1, 10.2, 11.6–11.9 and
Table A.7: STOP ends a session, Show Reset suppresses event mode until acknowledged,
and gesture/trackpad changes require their respective event-source enable bits.
Re-ATI completion is visible for one cycle. Unserviced windows expire at register
0x32's I2C timeout, discarding the window's gesture flags and pending report.
Current-sample finger count, coordinates, strength and area remain available to
later forced or verification reads: processing continues normally (§11.6).
Table A and §11.6 give no power-up timeout default; the model assumes 100 ms until
settings program it. Suspend takes effect at the end of the command session.

`input_backpressure` advances twelve alternating 500/501-X contact frames at a
sensor-owned 110-ms cadence, independent of reads. With one callback held and a
16-entry queue, eight packets produce 17 accepted reports; the ninth packet blocks
on X. Three later windows, carrying gestures as well as contact data, expire at
100 ms while the driver is blocked. After resuming, all 19 reports from the nine
serviced packets arrive in order. A fresh
lift and triple tap follow; the three lost sensor packets are never replayed.
`input_backpressure_verify_contact` keeps a stationary contact on a sensor with
120-ms verification and disabled tap mappings. Eight packets fill the 17 input
slots; an unmapped single-tap event
supplies the ninth packet and blocks its X report. Another unmapped tap window
expires while verification becomes queued. On resume, the ninth packet finishes,
and the due verification reads the still-held contact: exactly one down edge and
ten XY pairs (21 reports), with no release. A normal lift adds three reports.
The `timeout-invents-lift` mutation restores register zeroing in a disposable copy
and must fail this exact-stream check; it is a control, separate from product runs.

Unchanged contact frames in the filter cases use real touch verification reads;
explicit spurious RDY and stuck/held windows are fault injections. Finish checks
balanced buttons/contacts, no verification work, and no bus traffic for present
sensors during the 300-ms settle. Absent sensors may make only three boot attempts.

The entry retains case-list stdout/stderr/exit and checks both the owner identity
and resolved dependency selection before and after execution.
It normalizes the output path and rejects evidence inside the owner, workspace
or any selected west project before creating directories, including parent aliases
and uncloned project roots. External siblings and safe aliases remain supported;
discarded `..` components cannot create directories in a source. The runner
admission controls cover these paths and exact named-assertion rejection as well.

This fixture does not establish physical behaviour. The model has no capacitive
front end or gesture classifier, so whether a real part flags a triple tap remains
unmeasured. Forced communication completes immediately: clock stretching outside
a window (§11.9.2), the reseed after wake (§10.2), and electrical timing remain
unmodelled. It has no electrical bus, and native_sim time is not elapsed time on a
board. The GPIO emulator delivers edges only; there is no bus contention. Input
report failures cannot be injected through the real input queue, so those paths stay
with the host harnesses. Routing through a keyboard's input processors belongs to
the consuming keyboard. The pointer, mapped-button and relative sensors have real
listeners with no processors. `tap_button_map` also asserts twelve HID reports for
the single/double/triple taps on buttons 1/2/3, and `contact_rel` asserts five HID
reports with X 0, 0, 46, 46, 0, zero Y and no buttons. Each synced input event sends
one HID report, including zero motion. These checks reuse the existing cases.
`hid_abs_contact_is_left_button` pins the left-button edge a production route must
drop. Other cases retain their input expectations; the extra listeners only add HID
traces where their sensors emit input. The largest stream with the added listeners
is `two_sensors_independent` (15 HID reports), within `hid_seen`'s 128 entries.

## Raw tap-cell compile boundary

Run `python3 tests/integration/tap_ranges.py --workspace RESOLVED_WORKSPACE
--core stock-zmk|dya --output FRESH_ABSOLUTE_OUTPUT` on each supported core.
The entry resolves and checks the same pinned clean core/Zephyr projects as the
API fixture, stages unchanged owner source, configures the actual xiao_ble DTS,
and compiles only the actual IQS driver object. It retains commands, raw errors,
config/DTS and source hashes. Existing output paths stop before west is invoked.
Default selection retains all17 cases: explicit disabled cells for all3 taps;
default,0 and127 boundary instances; and each tap at-2,128,255,256 and65535.
`--case explicit-disabled` selects the valid sentinel regression alone.
Negative cases require exactly the named tap diagnostic; unrelated configuration
or compiler failures are failures. These are compile/API checks, not sensor,
input transport, physical gesture or power acceptance.

### Tap-result output isolation

The tap-range entry normalizes the output path before creating it. Evidence must
stay outside the owner, selected workspace and every west project root, including
projects not yet cloned. Parent symlinks cannot redirect output into an input.
A normalized external path is used for all writes, so discarded `..` components
do not create intermediate directories inside a source. Existing evidence remains
protected before dependency discovery. External sibling paths and safe parent
aliases remain supported.

`test_tap_output_roots.py`, included in canonical quick/full discovery, drives the
actual entry with synthetic source trees and controlled dependency/compiler
responses. It checks both core selections, direct and aliased owner/workspace/
manifest/core/Zephyr/HAL/uncloned roots, root equality, source byte/directory
preservation, normal external outputs, normalized existing targets and symlink
loops. Source/workspace rejection precedes west; selected external project
rejection follows discovery but precedes Git checks, staging and compilation.
These admission fixtures establish path isolation, not firmware acceptance.
The separate real supported-core tap/API/kernel entries retain their contracts.
