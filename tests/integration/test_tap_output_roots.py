# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
"""Admission checks use synthetic sources and controlled compiler responses."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


ENTRY = Path(__file__).with_name("tap_ranges.py")


class TapOutputRoots(unittest.TestCase):
    def load_entry(self):
        spec = importlib.util.spec_from_file_location("tap_ranges_under_test", ENTRY)
        entry = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(entry)
        return entry

    def fixture(self, base, core, listed_aliases=False):
        owner = base / "owner"
        workspace = base / "workspace"
        projects = {
            "manifest": base / "manifest",
            "zmk": base / "sources/zmk",
            "zephyr": base / "sources/zephyr",
            "hal": base / "dependencies/hal",
            "uncloned": base / "pending/dependency",
        }
        for path in (owner, workspace, *projects.values()):
            if path != projects["uncloned"]:
                path.mkdir(parents=True)
                (path / "sentinel").write_text("synthetic source: retain")
        integration = owner / "tests/integration"
        (integration / "tap-ranges").mkdir(parents=True)
        (integration / "tap-ranges/cases.json").write_text(
            json.dumps([{"name": "synthetic-valid"}]))
        pins = {"stock-zmk": "a" * 40, "dya": "b" * 40, "zephyr": "c" * 40}
        (integration / "dependencies.json").write_text(json.dumps(pins))
        aliases = base / "aliases"
        aliases.mkdir()
        roots = dict(owner=owner, workspace=workspace, **projects)
        for name, path in roots.items():
            (aliases / name).symlink_to(path, target_is_directory=True)
        listed = {name: aliases / name if listed_aliases else path
                  for name, path in projects.items()}
        return owner, workspace, roots, listed, pins, aliases

    def snapshot(self, roots):
        result = {}
        for name, root in roots.items():
            if not root.exists():
                result[name] = None
                continue
            entries = []
            for path in sorted(root.rglob("*")):
                relative = path.relative_to(root).as_posix()
                if path.is_symlink():
                    entries.append((relative, "link", str(path.readlink())))
                elif path.is_file():
                    entries.append((relative, "file",
                                    hashlib.sha256(path.read_bytes()).hexdigest()))
                else:
                    entries.append((relative, "directory"))
            result[name] = entries
        return result

    def invoke(self, entry, fixture, core, output):
        owner, workspace, roots, listed, pins, aliases = fixture
        del roots, aliases
        calls = []
        def check_output(command, **kwargs):
            calls.append(command)
            if command == ["west", "list", "-f", "{name}|{abspath}"]:
                return "".join(f"{name}|{path}\n" for name, path in listed.items())
            if command[0] == "git":
                name = next(name for name, path in listed.items()
                            if str(path) == command[2])
                if command[3:] == ["rev-parse", "HEAD"]:
                    return pins[core if name == "zmk" else name] + "\n"
                if command[3:] == ["status", "--porcelain"]:
                    return ""
            if command[0] == "ninja":
                return "fixture/src/iqs7211e.c.obj: C_COMPILER\n"
            raise AssertionError(f"unexpected dependency command: {command}")

        stderr, stdout = io.StringIO(), io.StringIO()
        argv = [str(ENTRY), "--workspace", str(workspace), "--core", core,
                "--output", str(output)]
        with mock.patch.object(entry, "ROOT", owner), \
             mock.patch.object(sys, "argv", argv), \
             mock.patch.object(entry.subprocess, "check_output", side_effect=check_output), \
             mock.patch.object(entry.subprocess, "run",
                               return_value=subprocess.CompletedProcess([], 0)) as compiler, \
             contextlib.redirect_stderr(stderr), contextlib.redirect_stdout(stdout):
            try:
                entry.main()
                code = 0
            except SystemExit as error:
                code = error.code
        return code, stderr.getvalue(), calls, compiler.call_count

    def test_source_and_dependency_children_are_rejected_before_writes(self):
        entry = self.load_entry()
        names = ("owner", "workspace", "manifest", "zmk", "zephyr", "hal", "uncloned")
        for core in ("stock-zmk", "dya"):
            for name in names:
                for alias in (False, True):
                    with self.subTest(core=core, root=name, parent_alias=alias), \
                         tempfile.TemporaryDirectory(prefix="tap-output-roots-") as folder:
                        base = Path(folder)
                        fixture = self.fixture(base, core, listed_aliases=alias)
                        _, _, roots, _, _, aliases = fixture
                        parent = aliases / name if alias else roots[name]
                        output = parent / "new-results/deep"
                        before = self.snapshot(roots)
                        code, stderr, calls, compiler_calls = self.invoke(
                            entry, fixture, core, output)
                        self.assertEqual(self.snapshot(roots), before, "source roots changed")
                        self.assertEqual(code, 2, stderr)
                        self.assertIn("outside", stderr)
                        self.assertFalse(output.exists())
                        self.assertEqual(compiler_calls, 0)
                        self.assertTrue(all(call[0] == "west" for call in calls), calls)

    def test_uncloned_project_root_itself_is_rejected(self):
        entry = self.load_entry()
        for core in ("stock-zmk", "dya"):
            with self.subTest(core=core), tempfile.TemporaryDirectory() as folder:
                fixture = self.fixture(Path(folder), core)
                roots = fixture[2]
                before = self.snapshot(roots)
                code, stderr, calls, compiler_calls = self.invoke(
                    entry, fixture, core, roots["uncloned"])
                self.assertEqual(self.snapshot(roots), before, "source roots changed")
                self.assertEqual(code, 2, stderr)
                self.assertIn("outside", stderr)
                self.assertEqual(compiler_calls, 0)
                self.assertTrue(all(call[0] == "west" for call in calls), calls)

    def test_external_siblings_and_parent_aliases_remain_supported(self):
        entry = self.load_entry()
        for core in ("stock-zmk", "dya"):
            for alias in (False, True):
                with self.subTest(core=core, parent_alias=alias), \
                     tempfile.TemporaryDirectory() as folder:
                    base = Path(folder)
                    fixture = self.fixture(base, core, listed_aliases=alias)
                    output = base / "sources/zmk-results"
                    if alias:
                        link = base / "output-alias"
                        link.symlink_to(output.parent, target_is_directory=True)
                        output = link / output.name
                    before = self.snapshot(fixture[2])
                    code, stderr, _, compiler_calls = self.invoke(entry, fixture, core, output)
                    self.assertEqual(code, 0, stderr)
                    self.assertEqual(self.snapshot(fixture[2]), before, "source roots changed")
                    self.assertEqual(compiler_calls, 2)
                    results = json.loads((output / "results.json").read_text())
                    self.assertEqual([row["status"] for row in results], ["PASS"])

    def test_normalized_external_output_does_not_create_source_parents(self):
        entry = self.load_entry()
        for core in ("stock-zmk", "dya"):
            with self.subTest(core=core), tempfile.TemporaryDirectory() as folder:
                base = Path(folder)
                fixture = self.fixture(base, core)
                output = fixture[0] / "unused/../../safe-results"
                before = self.snapshot(fixture[2])
                code, stderr, _, compiler_calls = self.invoke(entry, fixture, core, output)
                self.assertEqual(code, 0, stderr)
                self.assertEqual(self.snapshot(fixture[2]), before, "source roots changed")
                self.assertEqual(compiler_calls, 2)
                self.assertTrue((base / "safe-results/results.json").is_file())

    def test_normalized_existing_output_is_rejected_before_discovery(self):
        entry = self.load_entry()
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            fixture = self.fixture(base, "dya")
            existing = base / "previous-results"
            existing.mkdir()
            sentinel = existing / "sentinel"
            sentinel.write_text("retain previous evidence")
            output = fixture[0] / "unused/../../previous-results"
            before = self.snapshot(fixture[2])
            code, stderr, calls, compiler_calls = self.invoke(entry, fixture, "dya", output)
            self.assertEqual(code, 2, stderr)
            self.assertIn("fresh absolute path", stderr)
            self.assertEqual(self.snapshot(fixture[2]), before, "source roots changed")
            self.assertEqual(sentinel.read_text(), "retain previous evidence")
            self.assertEqual(calls, [])
            self.assertEqual(compiler_calls, 0)


    def test_output_parent_symlink_loop_is_rejected_before_discovery(self):
        entry = self.load_entry()
        with tempfile.TemporaryDirectory() as folder:
            base = Path(folder)
            fixture = self.fixture(base, "dya")
            loop = base / "loop"
            loop.symlink_to(loop, target_is_directory=True)
            before = self.snapshot(fixture[2])
            code, stderr, calls, compiler_calls = self.invoke(
                entry, fixture, "dya", loop / "new-results")
            self.assertEqual(code, 2, stderr)
            self.assertIn("cannot resolve output path", stderr)
            self.assertEqual(self.snapshot(fixture[2]), before)
            self.assertEqual(calls, [])
            self.assertEqual(compiler_calls, 0)


if __name__ == "__main__":
    unittest.main()
