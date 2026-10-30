#!/usr/bin/env python3
"""Compile raw tap cells through the real supported-core devicetree path."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def reject_source_output(parser, output, roots):
    """Keep evidence outside every resolved input, including parent aliases."""
    for root in roots:
        source = Path(root).resolve()
        if output == source or source in output.parents:
            parser.error("output must be outside owner, workspace and selected source roots")


def main():
    cases = json.loads((ROOT / "tests/integration/tap-ranges/cases.json").read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--core", required=True, choices=("stock-zmk", "dya"))
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--case", action="append", choices=[case["name"] for case in cases])
    args = parser.parse_args()
    if not args.output.is_absolute() or args.output.exists() or args.output.is_symlink():
        parser.error("output must be a fresh absolute path")
    try:
        args.output = args.output.resolve()
    except (OSError, RuntimeError) as error:
        parser.error(f"cannot resolve output path: {error}")
    if args.output.exists():
        parser.error("output must be a fresh absolute path")
    workspace = args.workspace.resolve()
    reject_source_output(parser, args.output, (ROOT, workspace))
    lock = json.loads((ROOT / "tests/integration/dependencies.json").read_text())
    listing = subprocess.check_output(
        ["west", "list", "-f", "{name}|{abspath}"], cwd=workspace, text=True
    )
    projects = dict(line.split("|", 1) for line in listing.splitlines())
    reject_source_output(parser, args.output, projects.values())
    dependencies = []
    for name, expected in (("zmk", lock[args.core]), ("zephyr", lock["zephyr"])):
        path = Path(projects[name])
        head = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
        status = subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True)
        if head != expected or status:
            raise SystemExit(f"unchanged selected {name} required: {path}")
        dependencies.append(dict(name=name, path=str(path), revision=head))
    modules = []
    for name, path_text in projects.items():
        path = Path(path_text)
        if name in ("zmk", "zephyr", "manifest") or name.startswith("zmk-"):
            continue
        if (path / "zephyr/module.yml").is_file() or (path / "zephyr/CMakeLists.txt").is_file():
            modules.append(str(path))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "dependencies.json").write_text(json.dumps(dependencies, indent=2))
    selected = [case for case in cases if args.case is None or case["name"] in args.case]
    results = []
    with tempfile.TemporaryDirectory(prefix="iqs-tap-ranges-") as folder:
        stage = Path(folder) / "module"
        shutil.copytree(ROOT, stage, ignore=shutil.ignore_patterns(
            ".git", "node_modules", "artifacts", "dist", ".build", "build", "__pycache__", ".claude"
        ), symlinks=True)
        hashes = {p.relative_to(stage).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in stage.rglob("*") if p.is_file() and not p.is_symlink()}
        (args.output / "source-inputs.json").write_text(json.dumps(hashes, indent=2))
        zephyr = Path(projects["zephyr"])
        env = dict(os.environ, ZEPHYR_BASE=str(zephyr),
                   Zephyr_DIR=str(zephyr / "share/zephyr-package/cmake"))
        for case in selected:
            destination = args.output / case["name"]
            destination.mkdir()
            build = Path(folder) / case["name"]
            command = ["west", "build", "-p=always", "--cmake-only", "-s",
                       str(Path(projects["zmk"]) / "app"), "-d", str(build),
                       "-b", "xiao_ble//zmk", "--", "-DSHIELD=tester_xiao",
                       "-DZMK_CONFIG=" + str(stage / "tests/integration/config"),
                       "-DZEPHYR_MODULES=" + ";".join(modules), "-DZMK_EXTRA_MODULES=" + str(stage),
                       "-DEXTRA_DTC_OVERLAY_FILE=" + str(stage / "tests/integration/tap-ranges" / (case["name"] + ".overlay"))]
            with (destination / "build.log").open("wb") as log:
                configured = subprocess.run(command, cwd=workspace, env=env, stdout=log, stderr=subprocess.STDOUT)
                compile_command = []
                compiled = None
                if configured.returncode == 0:
                    targets = subprocess.check_output(["ninja", "-C", str(build), "-t", "targets", "all"], text=True)
                    targets = [line.split(":", 1)[0] for line in targets.splitlines()
                               if line.split(":", 1)[0].endswith("/src/iqs7211e.c.obj")]
                    if len(targets) != 1:
                        raise AssertionError(f"expected one actual IQS driver object, got {targets}")
                    compile_command = ["ninja", "-C", str(build), "-j2", *targets]
                    compiled = subprocess.run(compile_command, cwd=workspace, env=env, stdout=log, stderr=subprocess.STDOUT)
            text = (destination / "build.log").read_text(errors="replace")
            errors = re.findall(r"^.*: error: (.*)$", text, re.MULTILINE)
            expected = case.get("error")
            passed = configured.returncode == 0 and compiled is not None and (
                (compiled.returncode == 0 and not errors) if expected is None else
                (compiled.returncode != 0 and errors == [expected]))
            for relative in ("zephyr/zephyr.dts", "zephyr/.config", "CMakeCache.txt", "compile_commands.json"):
                source = build / relative
                if source.is_file():
                    shutil.copyfile(source, destination / source.name)
            row = dict(case=case["name"], status="PASS" if passed else "FAIL", command=command,
                       compile_command=compile_command, configure_exit=configured.returncode,
                       compile_exit=compiled.returncode if compiled is not None else None,
                       expected_error=expected, actual_errors=errors)
            results.append(row)
            (args.output / "results.json").write_text(json.dumps(results, indent=2))
            print(f"{args.core} tap-range {case['name']}: {row['status']}", flush=True)
    if any(row["status"] != "PASS" for row in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
