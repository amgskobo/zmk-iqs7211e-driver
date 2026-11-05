# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
"""Admission controls for mutation scope, worker verdicts and retained evidence."""
from contextlib import contextmanager, ExitStack
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('events_runner', Path(__file__).with_name('run.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class RunnerControls(unittest.TestCase):
    @contextmanager
    def mutation_control(self, *, worker_failure=None, rebuild='changed', baseline_passed=True,
                         rejected=True, case_exception=False):
        """Exercise the real queue/judge with synthetic builds, never west or a binary."""
        with tempfile.TemporaryDirectory() as temp, ExitStack() as patches:
            root = Path(temp)/'root'
            catalogue = root/'tests/kernel/events'
            catalogue.mkdir(parents=True)
            faults = [dict(name=f'fault-{i}', file='driver.c', before='old();',
                           after=f'fault_{i}();', expect_case='oracle') for i in range(2)]
            (catalogue/'mutations.json').write_text(json.dumps(dict(mutations=faults)))
            (catalogue/'cases.json').write_text(json.dumps(dict(cases={'oracle': 'synthetic'})))
            out = Path(temp)/'out'
            out.mkdir()

            def worker_index(stage):
                return int(stage.parent.name.split('-')[2])

            def stage_owner(stage):
                if worker_failure in ('exception', 'system-exit') and worker_index(stage) == 0:
                    error = SystemExit if worker_failure == 'system-exit' else RuntimeError
                    raise error('synthetic staging exception')
                stage.mkdir()
                (stage/'driver.c').write_text('old();\n')

            def configure(stage, workspace, dependencies, modules, folder, log_path):
                log_path.write_text('synthetic pristine build\n')
                if worker_failure == 'build' and worker_index(stage) == 0:
                    return 7
                (folder/'zephyr').mkdir(parents=True)
                (folder/'zephyr/zmk.exe').write_bytes(b'pristine')
                (folder/'zephyr/.config').write_text('synthetic config\n')
                (folder/'zephyr/zephyr.dts').write_text('synthetic DTS\n')
                (folder/'CMakeCache.txt').write_text('synthetic cache\n')
                return 0

            def build(root, workspace, dependencies, modules, target):
                target.mkdir()
                binary = target/'zmk.exe'
                binary.write_bytes(b'baseline')
                return binary

            def incremental(command, **kwargs):
                folder = Path(command[2])
                kwargs['stdout'].write(b'synthetic incremental build\n')
                if rebuild == 'changed':
                    source = folder.parent/'owner/driver.c'
                    (folder/'zephyr/zmk.exe').write_bytes(source.read_bytes())
                return SimpleNamespace(returncode=9 if rebuild == 'failed' else 0)

            def run_case(binary, case, stem):
                if case_exception:
                    raise RuntimeError('synthetic case exception')
                return (1, b'IQSFAIL case=oracle: synthetic assertion\n') if rejected else (0, b'')

            patches.enter_context(patch.object(runner, 'ROOT', root))
            patches.enter_context(patch.object(runner, 'stage_owner', side_effect=stage_owner))
            patches.enter_context(patch.object(runner, 'configure', side_effect=configure))
            patches.enter_context(patch.object(runner, 'build', side_effect=build))
            patches.enter_context(patch.object(runner, 'listed_cases', return_value=[b'oracle']))
            baseline = patches.enter_context(patch.object(runner, 'run_variant',
                                                         return_value=baseline_passed))
            patches.enter_context(patch.object(runner.subprocess, 'run', side_effect=incremental))
            cases = patches.enter_context(patch.object(runner, 'run_case', side_effect=run_case))
            passed = runner.run_mutations([], 2, root, {}, [], out)
            baseline.assert_called_once()
            yield passed, json.loads((out/'results.json').read_text()), out, cases

    def test_failed_pristine_worker_cannot_hide_behind_queue_draining_worker(self):
        with self.mutation_control(worker_failure='build') as (passed, result, out, cases):
            self.assertFalse(passed)
            self.assertTrue(result['baseline_passed'])
            self.assertEqual(result['workers'][0], dict(worker=0, result='PRISTINE-BUILD-FAILED', exit=7))
            self.assertEqual(result['workers'][1]['result'], 'SUCCESS')
            self.assertEqual([row['result'] for row in result['mutations']], ['REJECTED'] * 2)
            self.assertEqual([row['worker'] for row in result['mutations']], [1, 1])
            self.assertEqual(cases.call_count, 2)
            self.assertEqual((out/'worker-0/build.exit').read_text(), '7')

    def test_worker_exception_before_dequeue_is_a_terminal_failure(self):
        for failure, exception in (('exception', 'RuntimeError'), ('system-exit', 'SystemExit')):
            with self.subTest(exception=exception), self.mutation_control(worker_failure=failure) as control:
                passed, result, out, cases = control
                self.assertFalse(passed)
                worker = result['workers'][0]
                self.assertEqual(worker['result'], 'EXCEPTION')
                self.assertIn('Traceback (most recent call last)', worker['traceback'])
                self.assertIn(exception+': synthetic staging exception', worker['traceback'])
                self.assertEqual(result['workers'][1]['result'], 'SUCCESS')
                self.assertEqual([row['result'] for row in result['mutations']], ['REJECTED'] * 2)
                self.assertEqual(cases.call_count, 2)

    def test_pristine_hash_after_rebuild_is_stale_and_cases_do_not_run(self):
        with self.mutation_control(rebuild='stale') as (passed, result, out, cases):
            self.assertFalse(passed)
            cases.assert_not_called()
            pristine_hash = hashlib.sha256(b'pristine').hexdigest()
            for row in result['mutations']:
                self.assertEqual(row['result'], 'STALE-BINARY')
                self.assertEqual(row['binary_sha256'], pristine_hash)
                self.assertEqual((out/row['mutation']/'zmk.exe').read_bytes(), b'pristine')

    def test_success_requires_baseline_workers_and_all_fault_rejections(self):
        with self.mutation_control() as (passed, result, out, cases):
            self.assertTrue(passed)
            self.assertTrue(result['baseline_passed'])
            self.assertEqual([worker['result'] for worker in result['workers']], ['SUCCESS'] * 2)
            pristine_hash = hashlib.sha256(b'pristine').hexdigest()
            for index, worker in enumerate(result['workers']):
                evidence = out/f'worker-{index}'
                self.assertEqual(worker['pristine_sha256'], pristine_hash)
                self.assertEqual((evidence/'binary.sha256').read_text(), pristine_hash)
                for name, value in (('.config', 'synthetic config\n'), ('zephyr.dts', 'synthetic DTS\n'),
                                    ('CMakeCache.txt', 'synthetic cache\n')):
                    self.assertEqual((evidence/name).read_text(), value)
            self.assertEqual(cases.call_count, 2)
            for index, row in enumerate(result['mutations']):
                self.assertEqual(row['result'], 'REJECTED')
                expected_hash = hashlib.sha256(f'fault_{index}();\n'.encode()).hexdigest()
                self.assertEqual(row['binary_sha256'], expected_hash)
                self.assertNotEqual(row['binary_sha256'], pristine_hash)
                self.assertFalse((out/row['mutation']/'zmk.exe').exists())
            self.assertEqual([call.args[1] for call in cases.call_args_list], ['oracle'] * 2)

    def test_failed_baseline_still_fails_with_successful_workers(self):
        with self.mutation_control(baseline_passed=False) as (passed, result, out, cases):
            self.assertFalse(passed)
            self.assertFalse(result['baseline_passed'])
            self.assertEqual([worker['result'] for worker in result['workers']], ['SUCCESS'] * 2)
            self.assertEqual([row['result'] for row in result['mutations']], ['REJECTED'] * 2)

    def test_failed_rebuild_retains_log_and_available_executable(self):
        with self.mutation_control(rebuild='failed') as (passed, result, out, cases):
            self.assertFalse(passed)
            cases.assert_not_called()
            for row in result['mutations']:
                self.assertEqual(row['result'], 'BUILD-FAILED')
                target = out/row['mutation']
                self.assertEqual((target/'build.log').read_text(), 'synthetic incremental build\n')
                self.assertEqual((target/'build.exit').read_text(), '9')
                self.assertEqual((target/'zmk.exe').read_bytes(), b'pristine')

    def test_survivor_retains_exact_rebuilt_executable(self):
        with self.mutation_control(rejected=False) as (passed, result, out, cases):
            self.assertFalse(passed)
            for index, row in enumerate(result['mutations']):
                self.assertEqual(row['result'], 'SURVIVED')
                binary = (out/row['mutation']/'zmk.exe').read_bytes()
                self.assertEqual(binary, f'fault_{index}();\n'.encode())
                self.assertEqual(row['binary_sha256'], hashlib.sha256(binary).hexdigest())

    def test_case_exception_retains_hash_executable_and_terminal_worker_status(self):
        with self.mutation_control(case_exception=True) as (passed, result, out, cases):
            self.assertFalse(passed)
            self.assertEqual([worker['result'] for worker in result['workers']], ['EXCEPTION'] * 2)
            for row in result['mutations']:
                self.assertEqual(row['result'], 'EXCEPTION')
                self.assertIn('RuntimeError: synthetic case exception', row['traceback'])
                self.assertEqual(result['workers'][row['worker']]['traceback'], row['traceback'])
                binary = (out/row['mutation']/'zmk.exe').read_bytes()
                self.assertEqual(row['binary_sha256'], hashlib.sha256(binary).hexdigest())

    def test_function_scope_and_exact_count(self):
        with tempfile.TemporaryDirectory() as temp:
            stage = Path(temp)
            path = stage/'driver.c'
            original = 'void first()\n{\n    old();\n}\nvoid second()\n{\n    old();\n}\n'
            path.write_text(original)
            fault = dict(name='scoped', file='driver.c', within='void first()\n{',
                         before='old()', after='new()')
            runner.apply_mutation(stage, fault)
            self.assertEqual(path.read_text(), original.replace('old()', 'new()', 1))
            path.write_text(original)
            with self.assertRaises(SystemExit):
                runner.apply_mutation(stage, dict(fault, count=2))
            self.assertEqual(path.read_text(), original)
            del fault['within']
            with self.assertRaises(SystemExit):
                runner.apply_mutation(stage, fault)
            self.assertEqual(path.read_text(), original)
            runner.apply_mutation(stage, dict(fault, count=2))
            self.assertEqual(path.read_text(), original.replace('old()', 'new()'))

    def test_ambiguous_function_anchor(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'driver.c'
            original = 'void first()\n{\nold();\n}\n' * 2
            path.write_text(original)
            with self.assertRaises(SystemExit):
                runner.apply_mutation(Path(temp), dict(name='ambiguous', file='driver.c',
                    within='void first()\n{', before='old()', after='new()', count=2))
            self.assertEqual(path.read_text(), original)

    def test_failed_case_list_is_retained_and_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            binary = out/'list-fails'
            binary.write_text('#!/bin/sh\nprintf "IQSCASE plausible\\n"\necho list-error >&2\nexit 7\n')
            binary.chmod(0o755)
            with self.assertRaises(SystemExit):
                runner.listed_cases(binary, out)
            self.assertEqual((out/'case-list.stdout').read_text(), 'IQSCASE plausible\n')
            self.assertEqual((out/'case-list.stderr').read_text(), 'list-error\n')
            self.assertEqual((out/'case-list.exit').read_text(), '7')

    def test_identity_covers_build_and_dependency_inputs(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for item in runner.SOURCES:
                path = root/item
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('before')
            before = runner.identity(root)
            self.assertEqual(set(before), set(runner.SOURCES))
            for item in runner.SOURCES:
                path = root/item
                path.write_text('after')
                self.assertNotEqual(before, runner.identity(root), item)
                path.write_text('before')

    def test_expected_cases_accepts_one_or_many(self):
        self.assertEqual(runner.expected_cases(dict(expect_case='a')), ['a'])
        self.assertEqual(runner.expected_cases(dict(expect_case=['a', 'b'])), ['a', 'b'])

    def test_catalogue_matches_are_unambiguous(self):
        import json
        import shutil
        with tempfile.TemporaryDirectory() as temp:
            stage = Path(temp)
            catalogue = runner.ROOT/'tests/kernel/events/mutations.json'
            for fault in json.loads(catalogue.read_text())['mutations']:
                source = runner.ROOT/fault['file']
                target = stage/fault['file']
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, target)
                runner.apply_mutation(stage, fault)
                self.assertNotEqual(source.read_bytes(), target.read_bytes(), fault['name'])


if __name__ == '__main__':
    unittest.main()
