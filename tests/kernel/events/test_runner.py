# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
"""Admission controls for mutation scope, worker verdicts and retained evidence."""
from contextlib import contextmanager, ExitStack
import hashlib
import importlib.util
import io
import json
import sys
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
                         rejected=True, case_exception=False, case_exit=None, case_stdout=None,
                         jobs=2, expect_case='oracle', build_calls=None, baseline_exception=False):
        """Exercise the real queue/judge with synthetic builds, never west or a binary."""
        with tempfile.TemporaryDirectory() as temp, ExitStack() as patches:
            root = Path(temp)/'root'
            catalogue = root/'tests/kernel/events'
            catalogue.mkdir(parents=True)
            faults = [dict(name=f'fault-{i}', file='driver.c', before='old();',
                           after=f'fault_{i}();', expect_case=expect_case) for i in range(2)]
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
                if build_calls is not None:
                    build_calls.append('worker')
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
                if build_calls is not None:
                    build_calls.append('baseline')
                target.mkdir()
                binary = target/'zmk.exe'
                binary.write_bytes(b'baseline')
                return binary

            def incremental(command, **kwargs):
                folder = Path(command[2])
                kwargs['stdout'].write(b'synthetic incremental build\n')
                binary = folder/'zephyr/zmk.exe'
                if rebuild == 'changed' or (rebuild == 'first-only' and
                                            binary.read_bytes() == b'pristine'):
                    source = folder.parent/'owner/driver.c'
                    binary.write_bytes(source.read_bytes())
                return SimpleNamespace(returncode=9 if rebuild == 'failed' else 0)

            def run_case(binary, case, stem):
                if case_exception:
                    raise RuntimeError('synthetic case exception')
                if case_exit is not None:
                    return case_exit, case_stdout
                return (1, b'IQSFAIL case=oracle line=1: synthetic assertion\n') if rejected else (0, b'')

            patches.enter_context(patch.object(runner, 'ROOT', root))
            patches.enter_context(patch.object(runner, 'stage_owner', side_effect=stage_owner))
            patches.enter_context(patch.object(runner, 'configure', side_effect=configure))
            patches.enter_context(patch.object(runner, 'build', side_effect=build))
            patches.enter_context(patch.object(runner, 'listed_cases', return_value=[b'oracle']))
            baseline = patches.enter_context(patch.object(runner, 'run_variant',
                side_effect=RuntimeError('synthetic baseline exception') if baseline_exception else None,
                return_value=baseline_passed))
            patches.enter_context(patch.object(runner.subprocess, 'run', side_effect=incremental))
            cases = patches.enter_context(patch.object(runner, 'run_case', side_effect=run_case))
            passed = runner.run_mutations([], jobs, root, {}, [], out)
            if jobs == 1 and worker_failure:
                baseline.assert_not_called()
            else:
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

    def test_previous_mutant_after_rebuild_is_stale_and_its_failure_cannot_be_reused(self):
        with self.mutation_control(rebuild='first-only', jobs=1) as control:
            passed, result, out, cases = control
            self.assertFalse(passed)
            first, second = result['mutations']
            self.assertEqual(first['result'], 'REJECTED')
            self.assertEqual(second['result'], 'STALE-BINARY')
            self.assertEqual(second['binary_sha256'], first['binary_sha256'])
            self.assertEqual(second['previous_binary_sha256'], first['binary_sha256'])
            self.assertEqual((out/'fault-1/zmk.exe').read_bytes(), b'fault_0();\n')
            self.assertEqual(cases.call_count, 1)

    def test_empty_expected_cases_are_rejected_before_building(self):
        build_calls = []
        with self.assertRaisesRegex(SystemExit, 'at least one expected case'):
            with self.mutation_control(expect_case=[], build_calls=build_calls):
                pass
        self.assertEqual(build_calls, [])

    def test_duplicate_expected_cases_are_rejected_before_building(self):
        build_calls = []
        with self.assertRaisesRegex(SystemExit, 'duplicate expected cases'):
            with self.mutation_control(expect_case=['oracle', 'oracle'], build_calls=build_calls):
                pass
        self.assertEqual(build_calls, [])

    def test_repeated_product_selection_preserves_the_first_execution_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            binary = out/'synthetic-fixture'
            binary.write_text(
                '#!/usr/bin/env python3\n'
                'import os\n'
                'from pathlib import Path\n'
                'counter = Path(__file__).with_name("calls")\n'
                'count = int(counter.read_text()) if counter.exists() else 0\n'
                'counter.write_text(str(count + 1))\n'
                f'print("IQSTRACE batch=" + str(count // {runner.REPEATS}))\n'
                'print("IQS kernel EVENTS PASS case=" + os.environ["IQS_KERNEL_CASE"])\n')
            binary.chmod(0o755)
            self.assertTrue(runner.run_variant(binary, ['oracle', 'oracle'], out))
            self.assertIn(b'IQSTRACE batch=0\n', (out/'oracle-1.stdout').read_bytes(),
                          'the first execution evidence was overwritten')
            self.assertEqual(int((out/'calls').read_text()), runner.REPEATS)
            rows = json.loads((out/'results.json').read_text())
            self.assertEqual([row['repeat'] for row in rows if 'repeat' in row],
                             list(range(1, runner.REPEATS + 1)))
            self.assertEqual(sum('identical_traces' in row for row in rows), 1)

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

    def test_baseline_reuses_a_retained_pristine_binary_without_an_extra_build(self):
        build_calls = []
        with self.mutation_control(build_calls=build_calls) as (passed, result, out, cases):
            self.assertTrue(passed)
            self.assertEqual(build_calls, ['worker', 'worker'])
            binary = (out/'baseline/zmk.exe').read_bytes()
            self.assertEqual(binary, b'pristine')
            self.assertEqual(result['baseline']['binary_sha256'], hashlib.sha256(binary).hexdigest())
            worker = result['workers'][result['baseline']['source_worker']]
            self.assertEqual(result['baseline']['binary_sha256'], worker['pristine_sha256'])

    def test_product_selection_cannot_pass_without_any_case(self):
        with tempfile.TemporaryDirectory() as temp:
            self.assertFalse(runner.run_variant(Path(temp)/'unused', [], Path(temp)))

    def test_small_fault_selection_does_not_build_idle_workers(self):
        build_calls = []
        with self.mutation_control(jobs=8, build_calls=build_calls) as (passed, result, out, cases):
            self.assertTrue(passed)
            self.assertEqual(len(result['workers']), 2)
            self.assertEqual(build_calls, ['worker', 'worker'])

    def test_all_pristine_builds_failing_leaves_no_blocked_baseline_waiter(self):
        with self.mutation_control(jobs=1, worker_failure='build') as (passed, result, out, cases):
            self.assertFalse(passed)
            self.assertFalse(result['baseline_passed'])
            self.assertEqual(result['baseline']['result'], 'NOT-RUN')
            self.assertEqual([row['result'] for row in result['mutations']], ['NOT-RUN'] * 2)
            cases.assert_not_called()

    def test_baseline_exception_is_retained_and_cannot_pass_with_rejected_faults(self):
        with self.mutation_control(baseline_exception=True) as (passed, result, out, cases):
            self.assertFalse(passed)
            self.assertFalse(result['baseline_passed'])
            self.assertEqual(result['baseline']['result'], 'EXCEPTION')
            self.assertIn('RuntimeError: synthetic baseline exception', result['baseline']['traceback'])
            self.assertEqual([row['result'] for row in result['mutations']], ['REJECTED'] * 2)

    def test_product_requires_one_exact_completion_line_and_nonempty_traces(self):
        marker = 'IQS kernel EVENTS PASS case=oracle\n'
        trace = 'IQSTRACE independent observation\n'
        for stdout, accepted in ((marker + trace, True), (marker, False),
                                 ('diagnostic ' + marker + trace, False),
                                 (marker + marker + trace, False),
                                 (marker + trace + 'IQSFAIL case=oracle line=1: failure\n', False)):
            with self.subTest(stdout=stdout), tempfile.TemporaryDirectory() as temp:
                out = Path(temp)
                binary = out/'synthetic-product'
                binary.write_text('#!/usr/bin/env python3\nprint(' + repr(stdout) + ', end="")\n')
                binary.chmod(0o755)
                self.assertEqual(runner.run_variant(binary, ['oracle'], out), accepted)

    def test_failed_baseline_still_fails_with_successful_workers(self):
        with self.mutation_control(baseline_passed=False) as (passed, result, out, cases):
            self.assertFalse(passed)
            self.assertFalse(result['baseline_passed'])
            self.assertEqual([worker['result'] for worker in result['workers']], ['SUCCESS'] * 2)
            self.assertEqual([row['result'] for row in result['mutations']], ['REJECTED'] * 2)

    def test_only_the_named_assertion_exit_qualifies_as_a_rejection(self):
        assertion = b'IQSFAIL case=oracle line=1: synthetic assertion\n'
        for code, stdout in ((124, assertion), (-6, assertion), (7, assertion),
                             (1, b'IQSFAIL case=oracle_extra line=1: unrelated case\n'),
                             (1, b'diagnostic mentions IQSFAIL case=oracle line=1: failure\n')):
            with self.subTest(code=code, stdout=stdout), \
                 self.mutation_control(case_exit=code, case_stdout=stdout) as control:
                passed, result, out, cases = control
                self.assertFalse(passed)
                for row in result['mutations']:
                    self.assertNotEqual(row['result'], 'REJECTED')
                    self.assertFalse(row['cases'][0]['rejected'])
                    self.assertTrue((out/row['mutation']/'zmk.exe').is_file())

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


class OutputRoots(unittest.TestCase):
    @contextmanager
    def fixture(self, *, core='dya'):
        """Drive main with synthetic inputs; no west, compiler or product binary."""
        with tempfile.TemporaryDirectory() as temp, ExitStack() as patches:
            base = Path(temp)
            roots = {name: base/name for name in ('owner', 'workspace', 'manifest', 'zmk',
                                                 'zephyr', 'hal', 'uncloned')}
            for name, path in roots.items():
                if name != 'uncloned':
                    path.mkdir()
                    (path/'sentinel').write_text('synthetic input: retain')
            owner = roots['owner']
            for item in runner.SOURCES:
                path = owner/item
                path.parent.mkdir(parents=True, exist_ok=True)
                if item == 'tests/kernel/events':
                    path.mkdir()
                    (path/'cases.json').write_text(json.dumps(dict(cases={'oracle': 'synthetic'})))
                else:
                    path.write_text('synthetic source')
            pins = {'stock-zmk': 'a'*40, 'dya': 'b'*40, 'zephyr': 'c'*40}
            (owner/'tests/integration/dependencies.json').write_text(json.dumps(pins))
            projects = {name: path for name, path in roots.items() if name not in ('owner', 'workspace')}

            def check_output(command, **kwargs):
                if command[0] == 'west':
                    return ''.join(f'{name}|{path}\n' for name, path in projects.items())
                if command[0] == 'git':
                    if command[3:] == ['status', '--porcelain']:
                        return ''
                    name = Path(command[2]).name
                    return pins[core if name == 'zmk' else name]+'\n'
                raise AssertionError(command)

            def build(root, workspace, dependencies, modules, target):
                target.mkdir()
                binary = target/'zmk.exe'
                binary.write_text('synthetic executable')
                return binary

            patches.enter_context(patch.object(runner, 'ROOT', owner))
            commands = patches.enter_context(patch.object(runner.subprocess, 'check_output',
                                                         side_effect=check_output))
            builder = patches.enter_context(patch.object(runner, 'build', side_effect=build))
            patches.enter_context(patch.object(runner, 'listed_cases', return_value=[b'oracle']))
            patches.enter_context(patch.object(runner, 'run_variant', return_value=True))
            yield base, roots, projects, commands, builder

    def snapshot(self, roots):
        return {name: [(p.relative_to(root).as_posix(),
                        p.read_bytes() if p.is_file() else None)
                       for p in sorted(root.rglob('*'))] if root.exists() else None
                for name, root in roots.items()}

    def invoke(self, roots, output, *, core='dya', negative_control=False):
        from contextlib import redirect_stderr
        argv = ['run.py', '--workspace', str(roots['workspace']), '--core', core,
                '--output', str(output)]
        if negative_control:
            argv += ['--negative-control', 'mutations']
        stderr = io.StringIO()
        with patch.object(sys, 'argv', argv), redirect_stderr(stderr):
            try:
                runner.main()
                code = 0
            except SystemExit as error:
                code = error.code
        return code, stderr.getvalue()

    def test_source_roots_and_parent_aliases_are_rejected_before_writes(self):
        for name in ('owner', 'workspace', 'manifest', 'zmk', 'zephyr', 'hal', 'uncloned'):
            for alias in (False, True):
                with self.subTest(root=name, alias=alias), self.fixture() as fixture:
                    base, roots, projects, commands, builder = fixture
                    parent = roots[name]
                    if alias:
                        parent = base/'alias'
                        parent.symlink_to(roots[name], target_is_directory=True)
                        if name in projects:
                            projects[name] = parent
                    output = parent/'new/deep'
                    before = self.snapshot(roots)
                    code, stderr = self.invoke(roots, output)
                    self.assertEqual(self.snapshot(roots), before, 'input roots changed')
                    self.assertEqual(code, 2, stderr)
                    self.assertIn('outside', stderr)
                    builder.assert_not_called()
                    self.assertTrue(all(call.args[0][0] == 'west' for call in commands.call_args_list))

    def test_uncloned_root_equality_is_rejected_before_writes(self):
        with self.fixture() as fixture:
            base, roots, projects, commands, builder = fixture
            before = self.snapshot(roots)
            code, stderr = self.invoke(roots, roots['uncloned'])
            self.assertEqual(self.snapshot(roots), before, 'input roots changed')
            self.assertEqual(code, 2, stderr)
            self.assertIn('outside', stderr)
            builder.assert_not_called()

    def test_normalized_external_output_preserves_discarded_source_parents(self):
        with self.fixture() as fixture:
            base, roots, projects, commands, builder = fixture
            before = self.snapshot(roots)
            code, stderr = self.invoke(roots, roots['owner']/'unused/../../safe-results')
            self.assertEqual(code, 0, stderr)
            self.assertEqual(self.snapshot(roots), before, 'input roots changed')
            self.assertTrue((base/'safe-results/source-after.json').is_file())
            builder.assert_called_once()

    def test_normalized_existing_output_and_symlink_loops_reject_before_discovery(self):
        for kind in ('existing', 'loop'):
            with self.subTest(kind=kind), self.fixture() as fixture:
                base, roots, projects, commands, builder = fixture
                if kind == 'existing':
                    previous = base/'previous'
                    previous.mkdir()
                    (previous/'sentinel').write_text('retain evidence')
                    output = roots['owner']/'unused/../../previous'
                else:
                    loop = base/'loop'
                    loop.symlink_to(loop, target_is_directory=True)
                    output = loop/'new'
                before = self.snapshot(roots)
                code, stderr = self.invoke(roots, output)
                self.assertEqual(self.snapshot(roots), before, 'input roots changed')
                self.assertEqual(code, 2, stderr)
                commands.assert_not_called()
                builder.assert_not_called()

    def test_external_sibling_and_parent_alias_remain_supported(self):
        for alias in (False, True):
            with self.subTest(alias=alias), self.fixture() as fixture:
                base, roots, projects, commands, builder = fixture
                output = base/'zmk-results'
                if alias:
                    link = base/'output-alias'
                    link.symlink_to(base, target_is_directory=True)
                    output = link/output.name
                before = self.snapshot(roots)
                code, stderr = self.invoke(roots, output)
                self.assertEqual(code, 0, stderr)
                self.assertEqual(self.snapshot(roots), before)
                self.assertTrue((output/'source-after.json').is_file())
                builder.assert_called_once()


class DependencySources(unittest.TestCase):
    @contextmanager
    def invocation(self, change=None, *, core='dya', negative_control=False, directory_link=False):
        """Run actual selection/final admission with controlled Git and build responses."""
        real_check_output = runner.subprocess.check_output
        with OutputRoots().fixture(core=core) as fixture:
            base, roots, projects, commands, builder = fixture
            module = roots['hal']
            (module/'zephyr').mkdir()
            (module/'zephyr/module.yml').write_text('name: synthetic-hal\n')
            (module/'driver.c').write_text('working override A\n')
            (module/'local header.h').write_text('untracked override A\n')
            external = base/'external.h'
            external.write_text('linked override A\n')
            (module/'link.h').symlink_to(external)
            (module/'submodule').mkdir()
            (module/'submodule/child.c').write_text('nested override A\n')
            (module/'ignored-cache').write_text('generated cache A\n')
            # Git supplies the inventory, including a deleted tracked file and
            # a gitlink. NUL separation preserves spaces and newlines in names.
            names = ['sentinel', 'zephyr/module.yml', 'driver.c', 'local header.h',
                     'link.h', 'deleted.h', 'submodule']
            (module/'line\nbreak.h').write_text('untracked newline A\n')
            names.append('line\nbreak.h')
            dirty_status = ' M driver.c\n?? local header.h\n?? link.h\n'
            if directory_link:
                headers = base/'plain-headers'
                headers.mkdir()
                (headers/'api.h').write_bytes(b'linked directory override A\n')
                (module/'headers').symlink_to(headers, target_is_directory=True)
                names.append('headers')
                dirty_status += '?? headers\n'
            original_command = commands.side_effect

            def check_output(command, **kwargs):
                if command[:3] == ['git', '-C', str(module)]:
                    if command[3:] == ['rev-parse', 'HEAD']:
                        return 'd'*40+'\n'
                    if command[3:] == ['status', '--porcelain']:
                        return dirty_status
                    if command[3:] == ['ls-files', '--cached', '--others', '--exclude-standard', '-z']:
                        return b'\0'.join(name.encode() for name in names)+b'\0'
                    raise AssertionError(command)
                if command == ['git', '-C', str(module/'submodule'), 'ls-files',
                               '--cached', '--others', '--exclude-standard', '-z']:
                    return b'child.c\0'
                if directory_link and command[:3] == ['git', '-C', str(module/'headers')]:
                    # Use real Git for a directory outside every work tree.
                    return real_check_output(command, **kwargs)
                return original_command(command, **kwargs)

            def observe(*args):
                if change is not None:
                    change(module, external)
                return True

            commands.side_effect = check_output
            with patch.object(runner, 'run_variant', side_effect=observe) as product, \
                 patch.object(runner, 'run_mutations', side_effect=observe) as mutations:
                out = base/'results'
                code, stderr = OutputRoots().invoke(roots, out, core=core,
                                                    negative_control=negative_control)
                self.assertEqual(product.call_count, int(not negative_control))
                self.assertEqual(mutations.call_count, int(negative_control))
                yield code, stderr, out, module, dirty_status

    def test_changed_dependency_bytes_fail_even_when_git_status_is_identical(self):
        def relink(module, external):
            other = external.with_name('other.h')
            other.write_bytes(external.read_bytes())
            (module/'link.h').unlink()
            (module/'link.h').symlink_to(other)

        changes = {
            'tracked': lambda module, external: (module/'driver.c').write_text('working override B\n'),
            'untracked': lambda module, external: (module/'local header.h').write_text('untracked override B\n'),
            'newline-name': lambda module, external: (module/'line\nbreak.h').write_text('untracked newline B\n'),
            'linked-content': lambda module, external: external.write_text('linked override B\n'),
            'link-target-same-bytes': relink,
            'nested-module': lambda module, external: (module/'submodule/child.c').write_text('nested override B\n'),
            'executable-mode': lambda module, external: (module/'driver.c').chmod(0o755),
        }
        for core in ('stock-zmk', 'dya'):
            for negative in (False, True):
                for name, change in changes.items():
                    with self.subTest(core=core, negative=negative, change=name), \
                         self.invocation(change, core=core, negative_control=negative) as result:
                        code, stderr, out, module, status = result
                        self.assertEqual(code, 1, 'dependency drift was accepted')
                        before = json.loads((out/'dependencies.json').read_text())
                        after = json.loads((out/'dependencies-after.json').read_text())
                        self.assertEqual(before['modules']['hal']['status'], status)
                        self.assertEqual(after['modules']['hal']['status'], status)
                        self.assertNotEqual(before, after)

    def test_unchanged_overrides_and_ignored_cache_changes_remain_accepted(self):
        def generated_cache(module, external):
            (module/'ignored-cache').write_text('generated cache B\n')

        for core in ('stock-zmk', 'dya'):
            for negative in (False, True):
                for change in (None, generated_cache):
                    with self.subTest(core=core, negative=negative, cache=change is not None), \
                         self.invocation(change, core=core, negative_control=negative) as result:
                        code, stderr, out, module, status = result
                        self.assertEqual(code, 0, stderr)
                        before = json.loads((out/'dependencies.json').read_text())
                        after = json.loads((out/'dependencies-after.json').read_text())
                        self.assertEqual(before, after)
                        files = before['modules']['hal']['files']
                        self.assertEqual(files['driver.c']['sha256'], hashlib.sha256(b'working override A\n').hexdigest())
                        self.assertEqual(files['local header.h']['sha256'], hashlib.sha256(b'untracked override A\n').hexdigest())
                        self.assertEqual(files['link.h']['link'], str(module.parent/'external.h'))
                        self.assertEqual(files['deleted.h'], {'missing': True})
                        self.assertIn('child.c', files['submodule']['files'])
                        self.assertNotIn('ignored-cache', files)

    def test_cyclic_directory_links_have_a_named_rejection(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'loop').symlink_to(root, target_is_directory=True)
            with patch.object(runner.subprocess, 'check_output', return_value=b'loop\0'):
                with self.assertRaisesRegex(ValueError, 'cyclic dependency directory'):
                    runner.working_tree_files(root)

    def test_plain_directory_links_are_accepted_and_drift_is_rejected(self):
        def edit_header(module, external):
            (module/'headers/api.h').write_bytes(b'linked directory override B\n')

        for core in ('stock-zmk', 'dya'):
            for negative in (False, True):
                for change in (None, edit_header):
                    with self.subTest(core=core, negative=negative, changed=change is not None):
                        try:
                            with self.invocation(change, core=core, negative_control=negative,
                                                 directory_link=True) as result:
                                code, stderr, out, module, status = result
                                self.assertEqual(code, int(change is not None), stderr)
                                before = json.loads((out/'dependencies.json').read_text())
                                after = json.loads((out/'dependencies-after.json').read_text())
                                self.assertEqual(before['modules']['hal']['status'], status)
                                self.assertEqual(after['modules']['hal']['status'], status)
                                files = before['modules']['hal']['files']['headers']['files']
                                self.assertEqual(files['api.h']['sha256'],
                                                 hashlib.sha256(b'linked directory override A\n').hexdigest())
                                self.assertEqual(before == after, change is None)
                        except runner.subprocess.CalledProcessError:
                            self.fail('plain directory links must be accepted and fingerprinted')

    def test_other_git_inventory_failures_remain_errors(self):
        with tempfile.TemporaryDirectory() as temp:
            for code, diagnostic in ((128, b'fatal: detected dubious ownership in repository'),
                                     (128, b'fatal: bad index file'),
                                     (128, b'fatal: bad index at /synthetic/not a git repository/index'),
                                     (1, b'fatal: not a git repository')):
                with self.subTest(code=code, diagnostic=diagnostic):
                    error = runner.subprocess.CalledProcessError(code, ['git'], stderr=diagnostic)
                    with patch.object(runner.subprocess, 'check_output', side_effect=error):
                        with self.assertRaises(runner.subprocess.CalledProcessError):
                            runner.working_tree_files(Path(temp))

    def test_uninitialized_gitlinks_record_empty_and_local_override_trees(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with patch.object(runner.subprocess, 'check_output', return_value=b'./\0') as git:
                self.assertEqual(runner.working_tree_files(root), {})
                (root/'local.h').write_bytes(b'local gitlink override\n')
                (root/'nested').mkdir()
                (root/'nested/driver.c').write_bytes(b'nested gitlink override\n')
                files = runner.working_tree_files(root)
                self.assertEqual(files['local.h']['sha256'], hashlib.sha256(b'local gitlink override\n').hexdigest())
                self.assertEqual(files['nested']['files']['driver.c']['sha256'],
                                 hashlib.sha256(b'nested gitlink override\n').hexdigest())
                self.assertEqual(git.call_count, 2, 'a gitlink must not query its parent index recursively')


if __name__ == '__main__':
    unittest.main()
