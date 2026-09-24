"""Compile actual driver cleanup functions against a fault-injecting input stub."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[2]
source = (root / "src/iqs7211e.c").read_text()
source_path = root / "src/iqs7211e.c"
names = ["iqs7211e_report_abs_coordinates", "iqs7211e_release_click",
         "iqs7211e_release_touch", "iqs7211e_begin_runtime_reinitialization",
         "iqs7211e_click_work_handler", "iqs7211e_report_rel_coordinates",
         "iqs7211e_abort_touch_after_report_failure",
         "iqs7211e_touch_verify_chain_alive",
         "iqs7211e_report_data", "iqs7211e_boot_kick_work_handler"]
functions = []
for name in names:
    # Definitions, rather than forward declarations; braces at column zero
    # delimit the driver functions under test.
    import re
    match = re.search(r"static (?:int|void|bool) " + name + r"\([^;]+?\)\n\{", source)
    if not match:
        raise RuntimeError(name)
    end = source.index("\n}\n", match.end()) + 3
    line = source.count("\n", 0, match.start()) + 1
    functions.append(f'#line {line} "{source_path}"\n' + source[match.start():end])
harness = (root / "tests/runtime/harness.c").read_text()
with tempfile.TemporaryDirectory(prefix="iqs-runtime-") as folder:
    unit = pathlib.Path(folder) / "test.c"
    unit.write_text(harness.replace("/* DRIVER_FUNCTIONS */",
                                    "\n".join(functions) + f'\n#line 1 "{unit}"\n'))
    for variant, flags in (
        ("optimized", ["-O2"]),
        ("sanitized", ["-O1", "-g", "-fno-omit-frame-pointer",
                       "-fsanitize=address,undefined", "-fno-sanitize-recover=all"]),
        ("coverage", ["-O0", "--coverage"]),
    ):
        binary = pathlib.Path(folder) / variant
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        *flags, "-I" + str(root / "src"), str(unit),
                        str(root / "src/iqs7211e_filter.c"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    result = subprocess.run(
        ["gcov", "-b", "-c", "-o", str(pathlib.Path(folder) / "coverage-test.gcno"),
         str(unit)], cwd=folder, check=True, capture_output=True, text=True)
    print(result.stdout, end="")
    source_report = result.stdout.split(f"File '{source_path}'", 1)[1]
    source_report = source_report.split("Creating ", 1)[0]
    lines = float(re.search(r"Lines executed:([0-9.]+)%", source_report).group(1))
    branches = float(re.search(r"Taken at least once:([0-9.]+)%", source_report).group(1))
    if lines < 65.0 or branches < 50.0:
        raise RuntimeError("IQS7211E fault-path coverage regressed below 65% lines / 50% branches")
