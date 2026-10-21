# Retired IQS measurements (2026-10-16)

User-directed removal covers report-read/input-admission cadence/duration,
periodic FLOW counters/output and the optional queue-stack high-water collector.
The boot IRQ-seen latch remains functional recovery state; it is not a counter
or periodic observation. Sensor filtering/report/gesture/PM/recovery remains.

The exact pre-removal source, standalone timing fixture, integration options,
runtime harnesses and assertion documentation are hash-verified in
`<external-archive>/measurement-retirement-v1/sources-and-tests.zip`.
`manifest.json` beside it records every archived file hash. Prior evidence is
retained with its original image/source identity; no snapshot was accepted.
The consuming integration evidence records the actual external archive location;
`<external-archive>` is the operator-selected root, outside this checkout.

| Retired assertions | Current disposition |
| --- | --- |
| `timing_test.c`: baseline/gap, thresholds, uptime/counter wrap, independence, failed admission and five-second gate | RETIRED, source archived; feature removed |
| Report harness timing counters around actual ABS/REL helpers | RETIRED; actual ordered report success/failure/lifecycle assertions retained |
| Driver FLOW counters/time gate and stack collector limits | RETIRED; RDY/reset/PM/recovery and report failure/release assertions retained |
| API/DT timing enable on both supported cores | Option removed; same four filtered/bypass absolute/relative API variants retained |

`test_retirement.py` prevents the removed source/configuration from returning;
its guard must reject the archived pre-removal source as a negative control.
It does not replace the retired measurement algorithm assertions. The canonical
host entry still runs every retained filter and driver lifecycle/fault scenario
under ASan/UBSan. Physical I2C/RDY/input/power/reset acceptance remains separate.
