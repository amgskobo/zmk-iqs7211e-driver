# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class FreshEvidence(unittest.TestCase):
    def test_api_and_kernel_reject_existing_and_linked_results_before_west(self):
        root = Path(__file__).resolve().parents[2]
        for entry in ('tests/integration/run.py', 'tests/kernel/run.py', 'tests/integration/tap_ranges.py'):
            for kind in ('existing','linked','dangling','relative'):
                with tempfile.TemporaryDirectory() as folder:
                    base = Path(folder)
                    old = base / 'old'; old.mkdir()
                    sentinel = old / 'sentinel'; sentinel.write_text('retain ordered results')
                    link = base / 'link'; link.symlink_to(old, target_is_directory=True)
                    target = base / 'missing-target'
                    dangling = base / 'dangling'; dangling.symlink_to(target, target_is_directory=True)
                    commands = base / 'bin'; commands.mkdir()
                    west = commands / 'west'
                    west.write_text('#!/bin/sh\necho called >> "$WEST_MARKER"\nexit 86\n')
                    west.chmod(0o755)
                    marker = base / 'west-calls'
                    env = dict(os.environ, PATH=str(commands)+os.pathsep+os.environ['PATH'],
                               WEST_MARKER=str(marker), PYTHONDONTWRITEBYTECODE='1')
                    output = {'existing':old,'linked':link,'dangling':dangling,'relative':Path('relative')}[kind]
                    with self.subTest(entry=entry, output=str(output)):
                        command = [sys.executable, str(root/entry), '--workspace',str(base),
                                   '--core','dya','--output',str(output)]
                        result = subprocess.run(command,env=env,cwd=base,capture_output=True,text=True,timeout=5)
                        self.assertEqual(result.returncode,2,result.stdout+result.stderr)
                        self.assertIn('fresh absolute path',result.stderr)
                        self.assertFalse(marker.exists())
                        self.assertFalse(target.exists())
                        self.assertEqual(sentinel.read_text(),'retain ordered results')


if __name__ == '__main__':
    unittest.main()
