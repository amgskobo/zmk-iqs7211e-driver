# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT

"""Exercise all production-driver scenarios in quick or full host mode.

Quick runs every fixture under ASan/UBSan. Full additionally executes the
optimized and measured coverage variants and retains the coverage gate below.

Each fixture names a source file, the functions whose lines and branches it
gates, and any further functions it only needs in order to compile. They are
lifted out of the source, under #line markers that keep their real file and
line numbers, into tests/runtime/<fixture>_harness.c in place of its
/* DRIVER_FUNCTIONS */ marker. The harness supplies stand-ins for Zephyr and
ZMK and drives the functions. Every line and branch inside a gated function
must run, measured from gcov's per-line report rather than its per-file total,
so a support function is compiled in without being held to it.

A name may end in ":2" to lift the second of two definitions -- the other
side of an #if -- and a name starting with "#define " lifts that macro.
"""

import pathlib
import re
import subprocess
import tempfile

import os

mode = os.environ.get("ZMK_TEST_MODE", "quick")
if mode not in ("quick", "full"):
    raise SystemExit("ZMK_TEST_MODE must be quick or full")
variants = (('optimized', ['-O2']), ('sanitized', ['-O1', '-g', '-fno-omit-frame-pointer', '-fsanitize=address,undefined', '-fno-sanitize-recover=all']), ('coverage', ['-O0', '--coverage']))
if mode == "quick":
    variants = tuple(v for v in variants if v[0] == "sanitized")

root = pathlib.Path(__file__).resolve().parents[2]
source_dir = root / "src"
# The coordinate filter has its own gate in tests/filter; it is linked, not lifted.
links = (source_dir / "iqs7211e_filter.c",)
# Sources with a coverage gate of their own.
elsewhere = ("iqs7211e_filter.c",)
# Contact, click and cleanup functions, driven through their own stubbed
# neighbours in reports_harness.c.
reports = (
    "iqs7211e_report_abs_coordinates",
    "iqs7211e_release_click",
    "iqs7211e_release_touch",
    "iqs7211e_begin_runtime_reinitialization",
    "iqs7211e_click_work_handler",
    "iqs7211e_report_rel_coordinates",
    "iqs7211e_abort_touch_after_report_failure",
    "iqs7211e_touch_verify_chain_alive",
    "iqs7211e_report_data",
    "iqs7211e_boot_kick_work_handler",
)
fixtures = (
    ("reports", "iqs7211e.c", reports, ()),
    # Everything else, against a model of the sensor's memory map.
    ("driver", "iqs7211e.c", (
        "iqs7211e_start_work_queue",
        "iqs7211e_submit_work",
        "iqs7211e_reschedule_work",
        "iqs7211e_rdy_recheck_delay",
        "iqs7211e_schedule_rdy_recheck",
        "iqs7211e_retry_rdy_recheck",
        "iqs7211e_enable_interrupt_and_recheck",
        "iqs7211e_init_state",
        "iqs7211e_get_product_num",
        "iqs7211e_read_info_flags",
        "iqs7211e_check_reset",
        "iqs7211e_sw_reset",
        "iqs7211e_set_suspend_state",
        "iqs7211e_sensor_resume_retry_delay",
        "iqs7211e_retry_sensor_resume",
        "iqs7211e_acknowledge_reset",
        "iqs7211e_run_ati",
        "iqs7211e_read_ati_active",
        "iqs7211e_queue_value_updates",
        "iqs7211e_set_event_mode",
        "iqs7211e_get_touchpad_event",
        "iqs7211e_get_bit",
        "iqs7211e_get_num_fingers_from_info_flags",
        "iqs7211e_get_num_fingers",
        "iqs7211e_write_defaults",
        "iqs7211e_read_bytes",
        "iqs7211e_write_bytes",
        "iqs7211e_work_handler",
        "iqs7211e_rdy_recheck_work_handler",
        "iqs7211e_queue_clicks",
        "iqs7211e_touch_verify_work_handler",
        "set_gpio_interrupt",
        "iqs7211e_gpio_callback",
        "iqs7211e_init",
        "iqs7211e_pm_release_work_handler",
        "iqs7211e_restore_after_failed_suspend",
        "iqs7211e_release_before_suspend",
        "iqs7211e_pm_action",
        "iqs7211e_early_pm_action",
        "iqs7211e_early_pm_init",
    ), reports),
)


def definition(source, name):
    """Return (first line, last line, text) of name's definition."""
    if name.startswith("#define "):
        start = source.index(name)
        end = source.index("\n\n", start) + 1
        first = source.count("\n", 0, start) + 1
        return first, first + source.count("\n", start, end - 1), source[start:end]
    name, _, nth = name.partition(":")
    skip = int(nth or 1) - 1
    for match in re.finditer(r"\b" + name + r"\s*\(", source):
        closing = source.find(")", match.end())
        if not source[closing + 1:].lstrip().startswith("{"):
            continue  # a call or a prototype
        start = source.rfind("\n", 0, match.start()) + 1
        if start == match.start():
            start = source.rfind("\n", 0, start - 1) + 1  # return type on its own line
        head = source[start:match.start()]
        if not head or head[0].isspace() or head.startswith("#"):
            continue  # a call inside a body or a macro
        if skip:
            skip -= 1
            continue
        end = body_end(source, source.index("{", closing))
        first = source.count("\n", 0, start) + 1
        return first, first + source.count("\n", start, end - 1), source[start:end]
    raise RuntimeError(f"missing definition: {name}")


def body_end(source, opening):
    """Index just past the brace that closes the one at opening.

    Braces inside comments and string or character literals do not count.
    """
    depth = 0
    i = opening
    while True:
        if source.startswith("/*", i):
            i = source.index("*/", i) + 2
            continue
        if source.startswith("//", i):
            i = source.index("\n", i)
            continue
        char = source[i]
        if char in "\"'":
            i += 1
            while source[i] != char:
                i += 2 if source[i] == "\\" else 1
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1


def misses(gcov_text, ranges):
    """Count lines and branches inside ranges; list what never ran."""
    lines = executed = branches = taken = 0
    missed = []
    current = None
    for row in gcov_text.splitlines():
        if row.startswith("branch"):
            if current is None:
                continue
            branches += 1
            if re.search(r"taken [1-9]", row):
                taken += 1
            else:
                missed.append(f"  {current[1]}: {current[2].strip()}\n    {row}")
            continue
        found = re.match(r"\s*([^:]+):\s*(\d+):(.*)", row)
        if not found:
            continue
        count, number, text = found.group(1).strip(), int(found.group(2)), found.group(3)
        inside = any(first <= number <= last for first, last in ranges)
        current = (count, number, text) if inside else None
        if not inside or count == "-":
            continue
        lines += 1
        if count.startswith(("#####", "=====")):
            missed.append(f"  {number}: {text.strip()}  (never ran)")
        else:
            executed += 1
    return lines, executed, branches, taken, missed


def ungated_functions():
    """Functions defined in the sources that no fixture gates.

    A file named in `elsewhere` has a gate of its own and is skipped, so a
    function added to any other source without a test fails this run.
    """
    gated = {}
    for _fixture, file_name, names, _support in fixtures:
        gated.setdefault(file_name, set()).update(name.split(":")[0] for name in names)
    definition_pattern = re.compile(
        r"^(?:static\s+|__weak\s+)?(?:inline\s+)?(?:const\s+)?[a-z_0-9]+(?:\s+[a-z_0-9]+)?"
        r"[\s*]+([a-z_0-9]+)\([^;]*?\)\s*\{", re.M)
    missing = []
    for path in sorted(source_dir.glob("*.c")):
        if path.name in elsewhere:
            continue
        defined = set(definition_pattern.findall(path.read_text(encoding="utf-8")))
        missing += [f"{path.name}: {name}" for name in sorted(defined - gated.get(path.name, set()))]
    return missing


missing_functions = ungated_functions()
if missing_functions:
    raise RuntimeError("functions no fixture gates:\n  " + "\n  ".join(missing_functions))

for fixture, file_name, gated, support, bypass in (
        (*row, bypass) for row in fixtures for bypass in (False, True)):
    source_path = source_dir / file_name
    source = source_path.read_text(encoding="utf-8")
    functions = []
    ranges = []
    for name in (*support, *gated):
        first, last, text = definition(source, name)
        if name in gated:
            ranges.append((first, last))
        elif not name.startswith("#"):
            # A harness need not call every function it only compiles.
            text = "__attribute__((unused)) " + text
        functions.append(f'#line {first} "{source_path}"\n{text}')

    harness_path = root / "tests/runtime" / f"{fixture}_harness.c"
    harness = harness_path.read_text(encoding="utf-8")
    marker = "/* DRIVER_FUNCTIONS */"
    resume = harness.count("\n", 0, harness.index(marker)) + 2
    label = fixture + ("-unfiltered" if bypass else "-filtered")
    with tempfile.TemporaryDirectory(prefix=f"{label}-runtime-") as folder:
        work = pathlib.Path(folder)
        unit = work / "test.c"
        unit.write_text(harness.replace(marker, "\n".join(functions) +
                                        f'\n#line {resume} "{harness_path}"'), encoding="utf-8")
        for variant, flags in variants:
            binary = work / variant
            feature_flags = ["-DCONFIG_IQS7211E_FILTER_BYPASS=1"] if bypass else []
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", *flags, *feature_flags,
                            "-I" + str(root / "include"), "-I" + str(source_dir),
                            "-I" + str(root / "tests/runtime"),
                            str(unit), *map(str, links), "-o", str(binary)],
                           check=True)
            subprocess.run([str(binary)], check=True)
        if mode == "quick":
            print(f"{label}: PASS (all lifecycle scenarios and selected output traces, ASan/UBSan)")
            continue
        subprocess.run(["gcov", "-b", "-c", "-o", str(work / "coverage-test.gcno"), str(unit)],
                       cwd=work, check=True, capture_output=True, text=True)
        report = (work / f"{file_name}.gcov").read_text(encoding="utf-8")
        lines, executed, branches, taken, missed = misses(report, ranges)
        count = sum(not name.startswith("#") for name in gated)
        print(f"{label}: {count} functions of {file_name}: "
              f"lines {executed}/{lines}, branches {taken}/{branches}")
        if missed:
            print("\n".join(missed))
            raise RuntimeError(f"{fixture} fell below 100% coverage")
