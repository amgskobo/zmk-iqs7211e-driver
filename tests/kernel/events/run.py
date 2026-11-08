#!/usr/bin/env python3
# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
"""Driver events on the real kernel against a modelled sensor; no physical acceptance."""
import argparse, difflib, hashlib, json, os, queue, re, shutil, stat, subprocess, tempfile, threading, time, traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
# One build at the production boards' tick rate.
BUILD_EXTRA = ('-DCONFIG_SYS_CLOCK_TICKS_PER_SEC=32768',)
VARIANT = 'ticks-32768'
REPEATS = 3
CASE_TIMEOUT = 60
TRACE_LINE = re.compile(rb'^IQS(?:TRACE|HID|BUS|RDY) .*$', re.M)
SOURCES = ('src', 'tests/kernel/events', 'Kconfig', 'dts', 'CMakeLists.txt',
           'zephyr', 'tests/integration/dependencies.json')
# What a mutation build needs from the owner tree.
STAGED = ('src', 'dts', 'Kconfig', 'CMakeLists.txt', 'zephyr', 'tests/kernel/events')


def reject_source_output(output, roots):
    """Resolve aliases before any evidence write can create a source directory."""
    for root in roots:
        source = Path(root).resolve()
        if output == source or source in output.parents:
            raise ValueError('output must be outside owner, workspace and selected source roots')


def working_tree_files(root, ancestors=(), *, git_inventory=True):
    """Fingerprint tracked and nonignored untracked inputs, including local overrides."""
    resolved = root.resolve()
    if resolved in ancestors:
        raise ValueError('cyclic dependency directory: '+str(root))
    if git_inventory:
        try:
            listing = subprocess.check_output(['git', '-C', str(root), 'ls-files',
                                               '--cached', '--others', '--exclude-standard', '-z'],
                                              stderr=subprocess.PIPE,
                                              env=dict(os.environ, LC_ALL='C'))
        except subprocess.CalledProcessError as error:
            if error.returncode != 128 or not (error.stderr or b'').startswith(b'fatal: not a git repository '):
                if error.stderr:
                    os.write(2, error.stderr)
                raise
            # A directory link can point to local source without any Git index.
            git_inventory = False
        else:
            names = set(listing.split(b'\0')) - {b''}
            # In an uninitialized gitlink Git finds the parent repository and lists
            # the gitlink itself as './'. Its local contents have no own Git index.
            if names & {b'.', b'./'}:
                git_inventory = False
    if not git_inventory:
        names = {os.fsencode(path.name) for path in root.iterdir() if path.name != '.git'}
    files = {}
    for name in sorted(names):
        relative = os.fsdecode(name)
        path = root/relative
        entry = {}
        if path.is_symlink():
            entry['link'] = os.readlink(path)
        try:
            info = path.stat()
        except FileNotFoundError:
            entry['missing'] = True
        else:
            if stat.S_ISDIR(info.st_mode):
                # Gitlinks and directory links have their own Git file inventory.
                entry['files'] = working_tree_files(path, (*ancestors, resolved),
                                                    git_inventory=git_inventory)
            elif stat.S_ISREG(info.st_mode):
                digest = hashlib.sha256()
                with path.open('rb') as source:
                    for chunk in iter(lambda: source.read(1024*1024), b''):
                        digest.update(chunk)
                entry.update(sha256=digest.hexdigest(), mode=stat.S_IMODE(info.st_mode))
            else:
                raise ValueError('unsupported dependency input: '+str(path))
        files[relative] = entry
    return files


def select_workspace(workspace, core, output=None):
    pins = json.loads((ROOT/'tests/integration/dependencies.json').read_text())
    dependencies = {}
    modules = []
    listing = subprocess.check_output(['west', 'list', '-f', '{name}|{abspath}'], cwd=workspace, text=True)
    if output is not None:
        reject_source_output(output, (line.split('|', 1)[1] for line in listing.splitlines()))
    dependencies['west-list'] = listing.splitlines()
    dependencies['modules'] = {}
    for line in listing.splitlines():
        name, text = line.split('|', 1)
        path = Path(text)
        if name in ('zmk', 'zephyr'):
            rev = subprocess.check_output(['git', '-C', text, 'rev-parse', 'HEAD'], text=True).strip()
            status = subprocess.check_output(['git', '-C', text, 'status', '--porcelain'], text=True)
            expected = pins[core if name == 'zmk' else 'zephyr']
            if rev != expected or status:
                raise SystemExit('unchanged selected dependency required: '+name)
            dependencies[name] = {'path': text, 'revision': rev}
        elif not name.startswith('zmk-') and name != 'manifest':
            if (path/'zephyr/module.yml').is_file() or (path/'zephyr/CMakeLists.txt').is_file():
                modules.append(text)
                rev = subprocess.check_output(['git', '-C', text, 'rev-parse', 'HEAD'], text=True).strip()
                status = subprocess.check_output(['git', '-C', text, 'status', '--porcelain'], text=True)
                dependencies['modules'][name] = {'path': text, 'revision': rev, 'status': status,
                                                 'files': working_tree_files(path)}
    return dependencies, modules


def identity(root):
    files = []
    for item in SOURCES:
        path = root/item
        files.extend(sorted(p for p in path.rglob('*') if p.is_file()) if path.is_dir() else [path])
    return {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in files}


def configure(root, workspace, dependencies, modules, folder, log_path):
    """A pristine build of root into folder. Returns the west exit code."""
    command = ['west', 'build', '-p=always', '-o=-j2', '-s', dependencies['zmk']['path']+'/app',
               '-d', str(folder), '-b', 'native_sim//zmk_test_mock', '--',
               '-DZMK_CONFIG='+str(root/'tests/kernel/events/config'),
               '-DZEPHYR_MODULES='+';'.join(modules),
               '-DZMK_EXTRA_MODULES='+str(root/'tests/kernel/events'), *BUILD_EXTRA]
    env = dict(os.environ, ZEPHYR_BASE=dependencies['zephyr']['path'],
               Zephyr_DIR=dependencies['zephyr']['path']+'/share/zephyr-package/cmake')
    log_path.with_name(log_path.stem+'-command.json').write_text(
        json.dumps({'command': command, 'cwd': str(workspace)}, indent=2))
    started = time.monotonic()
    with log_path.open('wb') as log:
        code = subprocess.run(command, cwd=workspace, env=env, stdout=log, stderr=subprocess.STDOUT).returncode
    log_path.with_name(log_path.stem+'-time.json').write_text(
        json.dumps(dict(seconds=time.monotonic()-started, exit=code), indent=2))
    return code


def build(root, workspace, dependencies, modules, out):
    """Build once into out; the binary and its config are retained there."""
    out.mkdir(parents=True)
    with tempfile.TemporaryDirectory(prefix='iqs-events-') as temp:
        folder = Path(temp)/'build'
        code = configure(root, workspace, dependencies, modules, folder, out/'build.log')
        (out/'build.exit').write_text(str(code))
        if code:
            return None
        for source, name in ((folder/'zephyr/zmk.exe', 'zmk.exe'), (folder/'zephyr/.config', 'config'),
                             (folder/'zephyr/zephyr.dts', 'zephyr.dts'), (folder/'CMakeCache.txt', 'CMakeCache.txt')):
            shutil.copy2(source, out/name)
    binary = out/'zmk.exe'
    (out/'binary.sha256').write_text(hashlib.sha256(binary.read_bytes()).hexdigest())
    return binary


def run_case(binary, case, stem):
    env = dict(os.environ, IQS_KERNEL_CASE=case)
    started_ns = time.time_ns()
    started = time.monotonic()
    try:
        result = subprocess.run([str(binary)], env=env, capture_output=True, timeout=CASE_TIMEOUT)
        code, stdout, stderr = result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired as error:
        code, stdout, stderr = 124, error.stdout or b'', error.stderr or b''
    Path(str(stem)+'.time.json').write_text(json.dumps(dict(
        start_unix_ns=started_ns, seconds=time.monotonic()-started, exit=code), indent=2))
    Path(str(stem)+'.stdout').write_bytes(stdout)
    Path(str(stem)+'.stderr').write_bytes(stderr)
    Path(str(stem)+'.exit').write_text(str(code))
    return code, stdout


def listed_cases(binary, out):
    code, output = run_case(binary, '--list', out/'case-list')
    if code:
        raise SystemExit('case listing failed: '+str(code))
    return re.findall(rb'^IQSCASE (\S+)', output, re.M)


def run_variant(binary, cases, out):
    """Every case REPEATS times on one binary; each must pass with one identical trace."""
    rows = []
    for case in dict.fromkeys(cases):
        traces = set()
        passed = True
        for repeat in range(1, REPEATS+1):
            code, stdout = run_case(binary, case, out/f'{case}-{repeat}')
            trace = tuple(TRACE_LINE.findall(stdout))
            completions = stdout.splitlines().count(('IQS kernel EVENTS PASS case='+case).encode())
            valid = (code == 0 and completions == 1 and bool(trace)
                     and re.search(rb'^IQSFAIL\b', stdout, re.M) is None)
            traces.add(trace)
            rows.append(dict(case=case, repeat=repeat, exit=code, pass_verified=valid,
                             completion_count=completions, trace_lines=len(trace)))
            passed = passed and valid
        deterministic = len(traces) == 1
        status = 'PASS' if passed and deterministic else 'FAIL'
        note = '' if deterministic else ' (traces differ between identical runs)'
        print(f'{out.name} {case}: {status}{note}', flush=True)
        rows.append(dict(case=case, identical_traces=deterministic))
    (out/'results.json').write_text(json.dumps(rows, indent=2))
    return bool(rows) and all(row.get('pass_verified', row.get('identical_traces')) for row in rows)


def expected_cases(mutation):
    """One case name, or a list of them; every one must reject the fault."""
    cases = mutation['expect_case']
    names = [cases] if isinstance(cases, str) else list(cases)
    if not names:
        raise SystemExit('at least one expected case required: '+mutation.get('name', '<unnamed>'))
    if len(names) != len(set(names)):
        raise SystemExit('duplicate expected cases: '+mutation.get('name', '<unnamed>'))
    return names


def apply_mutation(stage, mutation):
    path = stage/mutation['file']
    text = path.read_text()
    start, end = 0, len(text)
    if 'within' in mutation:
        anchor = mutation['within']
        if text.count(anchor) != 1:
            raise SystemExit('ambiguous function anchor: '+mutation['name'])
        start = text.index(anchor)
        closing = re.search(r'^}\r?$', text[start:], re.M)
        if closing is None:
            raise SystemExit('missing function end: '+mutation['name'])
        end = start + closing.end()
    body = text[start:end]
    expected = mutation.get('count', 1)
    if expected < 1 or body.count(mutation['before']) != expected:
        raise SystemExit('mutation match count differs: '+mutation['name'])
    path.write_text(text[:start]+body.replace(mutation['before'], mutation['after'])+text[end:])


def stage_owner(stage):
    for item in STAGED:
        source = ROOT/item
        if source.is_dir():
            shutil.copytree(source, stage/item)
        else:
            (stage/item).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, stage/item)


def judge_mutation(stage, folder, mutation, target, pristine_hash):
    """Apply one fault to the worker's tree, rebuild incrementally, run its cases, restore."""
    name = mutation['name']
    path = stage/mutation['file']
    original = path.read_text()
    binary = folder/'zephyr/zmk.exe'
    row = dict(mutation=name, result='EXCEPTION')
    started = time.monotonic()
    try:
        row['previous_binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
        try:
            apply_mutation(stage, mutation)
        except SystemExit as error:
            row.update(result='NOT-APPLIED', detail=str(error))
            return row
        (target/'mutation.diff').write_text(''.join(difflib.unified_diff(
            original.splitlines(True), path.read_text().splitlines(True),
            'a/'+mutation['file'], 'b/'+mutation['file'])))
        build_started = time.monotonic()
        with (target/'build.log').open('wb') as log:
            code = subprocess.run(['cmake', '--build', str(folder)], stdout=log,
                                  stderr=subprocess.STDOUT).returncode
        (target/'build.exit').write_text(str(code))
        row['build_seconds'] = time.monotonic() - build_started
        if code:
            row['result'] = 'BUILD-FAILED'
            return row
        row['binary_sha256'] = hashlib.sha256(binary.read_bytes()).hexdigest()
        if row['binary_sha256'] in (pristine_hash, row['previous_binary_sha256']):
            row['result'] = 'STALE-BINARY'
            return row
        rows = []
        cases_started = time.monotonic()
        for case in expected_cases(mutation):
            code, stdout = run_case(binary, case, target/case)
            assertion = re.compile(rb'^IQSFAIL case='+re.escape(case.encode())+rb' line=[0-9]+: .+$')
            first = next((line for line in stdout.splitlines() if assertion.fullmatch(line)), b'').decode()
            rows.append(dict(case=case, exit=code, rejected=code == 1 and bool(first), line=first))
        rejected = all(row['rejected'] for row in rows)
        row['case_seconds'] = time.monotonic() - cases_started
        row.update(result='REJECTED' if rejected else 'SURVIVED', cases=rows)
        return row
    except BaseException:
        # Keep the successful rebuild's hash even if case execution raises.
        row.update(result='EXCEPTION', traceback=traceback.format_exc())
        return row
    finally:
        try:
            if row['result'] != 'REJECTED' and binary.is_file():
                shutil.copy2(binary, target/'zmk.exe')
        finally:
            path.write_text(original)
            row['seconds'] = time.monotonic() - started


def run_mutations(selected, jobs, workspace, dependencies, modules, out):
    """Each fault must build, and every named case must reject it by assertion."""
    started = time.monotonic()
    mutations = json.loads((ROOT/'tests/kernel/events/mutations.json').read_text())['mutations']
    if selected:
        mutations = [m for m in mutations if m['name'] in selected]
    if selected and set(selected) - {m['name'] for m in mutations}:
        raise SystemExit('unknown mutation')
    names = [m['name'] for m in mutations]
    if len(names) != len(set(names)):
        raise SystemExit('duplicate mutation names')
    if not mutations or jobs < 1:
        raise SystemExit('at least one mutation and a positive job count required')
    required = sorted(json.loads((ROOT/'tests/kernel/events/cases.json').read_text())['cases'])
    for mutation in mutations:
        if not set(expected_cases(mutation)) <= set(required):
            raise SystemExit('unknown expected case: '+mutation['name'])

    pending = queue.Queue()
    for mutation in mutations:
        pending.put(mutation)
    results = {}
    lock = threading.Lock()
    baseline_ok = []
    baseline_details = dict(result='NOT-RUN')
    baseline_ready = threading.Event()
    worker_statuses = [None] * min(jobs, len(mutations))

    def baseline():
        # Execute an immutable copy of the first successful pristine worker.
        # The worker can mutate its own executable while this copy runs.
        baseline_ready.wait()
        target = out/'baseline'
        began = time.monotonic()
        try:
            if baseline_details['result'] != 'READY':
                baseline_ok.append(False)
                return
            binary = target/'zmk.exe'
            if sorted(case.decode() for case in listed_cases(binary, target)) != required:
                baseline_details.update(result='CASE-LIST-MISMATCH')
                baseline_ok.append(False)
                print('baseline: the fixture and cases.json name different cases', flush=True)
                return
            cases = sorted({case for m in mutations for case in expected_cases(m)})
            passed = run_variant(binary, cases, target)
            baseline_ok.append(passed)
            baseline_details.update(result='PASS' if passed else 'FAIL')
        except BaseException:
            baseline_details.update(result='EXCEPTION', traceback=traceback.format_exc())
        finally:
            baseline_details['seconds'] = time.monotonic() - began

    def worker(index):
        status = dict(worker=index, result='SUCCESS')
        try:
            evidence = out/f'worker-{index}'
            evidence.mkdir()
            with tempfile.TemporaryDirectory(prefix=f'iqs-mutation-{index}-') as temp:
                stage = Path(temp)/'owner'
                folder = Path(temp)/'build'
                stage_started = time.monotonic()
                stage_owner(stage)
                (evidence/'stage-time.json').write_text(json.dumps(dict(
                    seconds=time.monotonic()-stage_started), indent=2))
                code = configure(stage, workspace, dependencies, modules, folder, evidence/'build.log')
                (evidence/'build.exit').write_text(str(code))
                if code:
                    status.update(result='PRISTINE-BUILD-FAILED', exit=code)
                    print(f'worker {index}: pristine build failed ({code})', flush=True)
                    return
                pristine_hash = hashlib.sha256((folder/'zephyr/zmk.exe').read_bytes()).hexdigest()
                status['pristine_sha256'] = pristine_hash
                (evidence/'binary.sha256').write_text(pristine_hash)
                for source, name in ((folder/'zephyr/.config', '.config'),
                                     (folder/'zephyr/zephyr.dts', 'zephyr.dts'),
                                     (folder/'CMakeCache.txt', 'CMakeCache.txt')):
                    shutil.copy2(source, evidence/name)
                with lock:
                    if not baseline_ready.is_set():
                        try:
                            target = out/'baseline'
                            target.mkdir()
                            for source, name in ((folder/'zephyr/zmk.exe', 'zmk.exe'),
                                                 (evidence/'.config', 'config'),
                                                 (evidence/'zephyr.dts', 'zephyr.dts'),
                                                 (evidence/'CMakeCache.txt', 'CMakeCache.txt')):
                                shutil.copy2(source, target/name)
                            for name in ('build.log', 'build.exit', 'build-command.json', 'build-time.json'):
                                if (evidence/name).is_file():
                                    shutil.copy2(evidence/name, target/name)
                            (target/'binary.sha256').write_text(pristine_hash)
                            baseline_details.update(result='READY', source_worker=index,
                                                    binary_sha256=pristine_hash)
                        except BaseException:
                            baseline_details.update(result='EXCEPTION', traceback=traceback.format_exc())
                            raise
                        finally:
                            baseline_ready.set()
                while True:
                    try:
                        mutation = pending.get_nowait()
                    except queue.Empty:
                        break
                    target = out/mutation['name']
                    target.mkdir()
                    row = judge_mutation(stage, folder, mutation, target, pristine_hash)
                    row['worker'] = index
                    with lock:
                        results[mutation['name']] = row
                    if row['result'] == 'EXCEPTION':
                        status.update(result='EXCEPTION', traceback=row['traceback'])
                        break
                    lines = ' '.join(case['line'] for case in row.get('cases', []))
                    print(f"{mutation['name']}: {row['result']} {lines}".rstrip(), flush=True)
        except BaseException:
            # SystemExit from admission helpers must also become a terminal verdict.
            status.update(result='EXCEPTION', traceback=traceback.format_exc())
        finally:
            with lock:
                worker_statuses[index] = status
                if all(status is not None for status in worker_statuses):
                    baseline_ready.set()  # No successful build must not leave a waiter blocked.

    threads = [threading.Thread(target=baseline)]
    threads += [threading.Thread(target=worker, args=(i,)) for i in range(len(worker_statuses))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    rows = [results.get(name, dict(mutation=name, result='NOT-RUN')) for name in names]
    (out/'results.json').write_text(json.dumps(dict(
        baseline_passed=baseline_ok == [True], baseline=baseline_details,
        workers=worker_statuses, mutations=rows,
        seconds=time.monotonic() - started), indent=2))
    counts = {}
    for row in rows:
        counts[row['result']] = counts.get(row['result'], 0) + 1
    print('mutations: '+', '.join(f'{key} {value}' for key, value in sorted(counts.items())), flush=True)
    # A baseline thread that stopped early leaves no verdict, which is a failure too.
    return (bool(rows) and baseline_ok == [True]
            and all(status is not None and status['result'] == 'SUCCESS' for status in worker_statuses)
            and all(row['result'] == 'REJECTED' for row in rows))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--core', required=True, choices=('stock-zmk', 'dya'))
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--case', action='append', default=[], help='run only this case; repeatable')
    parser.add_argument('--negative-control', choices=('mutations',))
    parser.add_argument('--mutation', action='append', default=[], help='with mutations: only this one; repeatable')
    parser.add_argument('--jobs', type=int, default=4, help='with mutations: parallel build trees (default 4)')
    args = parser.parse_args()
    out = args.output
    if not out.is_absolute() or out.exists() or out.is_symlink():
        parser.error('output must be a fresh absolute path')
    try:
        out = out.resolve()
    except (OSError, RuntimeError) as error:
        parser.error(f'cannot resolve output path: {error}')
    if out.exists():
        parser.error('output must be a fresh absolute path')
    workspace = args.workspace.resolve()
    try:
        reject_source_output(out, (ROOT, workspace))
        dependencies, modules = select_workspace(workspace, args.core, output=out)
    except ValueError as error:
        parser.error(str(error))
    out.mkdir(parents=True, exist_ok=False)
    (out/'dependencies.json').write_text(json.dumps(dependencies, indent=2))
    before = identity(ROOT)
    (out/'source-before.json').write_text(json.dumps(before, indent=2))

    if args.negative_control:
        passed = run_mutations(args.mutation, args.jobs, workspace, dependencies, modules, out)
    else:
        required = sorted(json.loads((ROOT/'tests/kernel/events/cases.json').read_text())['cases'])
        unknown = set(args.case) - set(required)
        if unknown:
            parser.error('unknown case: '+', '.join(sorted(unknown)))
        target = out/VARIANT
        binary = build(ROOT, workspace, dependencies, modules, target)
        if binary is None:
            print((target/'build.log').read_text()[-5000:])
            raise SystemExit(1)
        if sorted(case.decode() for case in listed_cases(binary, target)) != required:
            raise SystemExit('the fixture and cases.json name different cases')
        passed = run_variant(binary, args.case or required, target)

    after_dependencies, after_modules = select_workspace(workspace, args.core)
    (out/'dependencies-after.json').write_text(json.dumps(after_dependencies, indent=2))
    selection_unchanged = dependencies == after_dependencies and modules == after_modules
    after = identity(ROOT)
    (out/'source-after.json').write_text(json.dumps(after, indent=2))
    if before != after or not selection_unchanged or not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
