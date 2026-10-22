# Module-owned tests

Run from this checkout. Docker + Bash is the host entry point; no keyboard
checkout, shared test dispatcher or simulator repository is required.

```bash
bash tests/run-host-docker.sh
ZMK_TEST_MODE=full bash tests/run-host-docker.sh
```

On PowerShell, set `$env:ZMK_TEST_MODE="full"` for the second command
and invoke Git Bash explicitly. Remove the variable for the default quick run.

Quick executes **all behavioral fixtures and scenarios** once with
AddressSanitizer/UndefinedBehaviorSanitizer. Full repeats those same assertions
under the retained optimization, coverage and (where already present) 32-bit
variants. CI uses quick for push/PR and full for scheduled/manual runs.
Unknown mode values fail; there is no silent skipped success.

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
