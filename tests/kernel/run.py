#!/usr/bin/env python3
# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
"""Real work-cancellation boundary; no physical I2C or sensor acceptance."""
import argparse, hashlib, json, os, shutil, subprocess, tempfile
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--core', required=True, choices=('stock-zmk', 'dya'))
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--negative-control', choices=('invalid-priority',))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.output
    if not out.is_absolute() or out.exists() or out.is_symlink():
        parser.error('output must be a fresh absolute path')
    out.mkdir(parents=True, exist_ok=False)
    workspace = args.workspace.resolve()
    pins = json.loads((root/'tests/integration/dependencies.json').read_text())
    dependencies = {}
    modules = []
    listing = subprocess.check_output(['west','list','-f','{name}|{abspath}'],cwd=workspace,text=True)
    for line in listing.splitlines():
        name, text = line.split('|', 1)
        path = Path(text)
        if name in ('zmk','zephyr'):
            rev = subprocess.check_output(['git','-C',text,'rev-parse','HEAD'],text=True).strip()
            status = subprocess.check_output(['git','-C',text,'status','--porcelain'],text=True)
            expected = pins[args.core if name == 'zmk' else 'zephyr']
            if rev != expected or status:
                raise SystemExit('unchanged selected dependency required: '+name)
            dependencies[name] = {'path':text,'revision':rev}
        elif not name.startswith('zmk-') and name != 'manifest':
            if (path/'zephyr/module.yml').is_file() or (path/'zephyr/CMakeLists.txt').is_file():
                modules.append(text)
    (out/'dependencies.json').write_text(json.dumps(dependencies,indent=2))
    def identity():
        return {p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                for folder in ('src','tests/kernel') for p in sorted((root/folder).rglob('*')) if p.is_file()}
    before = identity()
    (out/'source-before.json').write_text(json.dumps(before,indent=2))
    with tempfile.TemporaryDirectory(prefix='iqs-kernel-') as temp:
        build = Path(temp)/'build'
        command = ['west','build','-p=always','-o=-j2','-s',dependencies['zmk']['path']+'/app',
                   '-d',str(build),'-b','native_sim//zmk_test_mock','--',
                   '-DZMK_CONFIG='+str(root/'tests/kernel/config'),
                   '-DZEPHYR_MODULES='+';'.join(modules),
                   '-DZMK_EXTRA_MODULES='+str(root/'tests/kernel/module')]
        if args.negative_control:
            command += ['-DCONFIG_NUM_PREEMPT_PRIORITIES=15',
                        '-DCONFIG_IQS7211E_WORKQUEUE_PRIORITY=15']
        env = dict(os.environ,ZEPHYR_BASE=dependencies['zephyr']['path'],
                   Zephyr_DIR=dependencies['zephyr']['path']+'/share/zephyr-package/cmake')
        (out/'build-command.json').write_text(json.dumps({'command':command,'cwd':str(workspace)},indent=2))
        with (out/'build.log').open('wb') as log:
            result = subprocess.run(command,cwd=workspace,env=env,stdout=log,stderr=subprocess.STDOUT)
        (out/'build.exit').write_text(str(result.returncode))
        if args.negative_control:
            valid = result.returncode != 0 and 'IQS work queue priority must fit configured Zephyr priorities' in (out/'build.log').read_text()
            (out/'negative-control.json').write_text(json.dumps(dict(control=args.negative_control,build_exit=result.returncode,expected_failure_verified=valid),indent=2))
            if (build/'zephyr/.config').is_file(): shutil.copy2(build/'zephyr/.config',out/'config')
            (out/'source-after.json').write_text(json.dumps(identity(),indent=2))
            print('invalid-priority named rejection:', 'PASS' if valid else 'FAIL',flush=True)
            raise SystemExit(0 if valid else 1)
        if result.returncode:
            print((out/'build.log').read_text()[-5000:]);raise SystemExit(result.returncode)
        for source, name in ((build/'zephyr/zmk.exe','zmk.exe'),(build/'zephyr/.config','config'),
                             (build/'zephyr/zephyr.dts','zephyr.dts'),(build/'CMakeCache.txt','CMakeCache.txt')):
            shutil.copy2(source,out/name)
    binary = out/'zmk.exe'
    (out/'binary.sha256').write_text(hashlib.sha256(binary.read_bytes()).hexdigest())
    rows = []
    for case in ('work','click','input'):
        for repeat in (1,2,3):
            env = dict(os.environ,IQS_KERNEL_CASE=case)
            try:
                result = subprocess.run([str(binary)],env=env,capture_output=True,timeout=5)
                code, stdout, stderr = result.returncode,result.stdout,result.stderr
            except subprocess.TimeoutExpired as error:
                code, stdout, stderr = 124,error.stdout or b'',error.stderr or b''
            stem = f'{case}-{repeat}'
            (out/(stem+'.stdout')).write_bytes(stdout)
            (out/(stem+'.stderr')).write_bytes(stderr)
            (out/(stem+'.exit')).write_text(str(code))
            valid = code == 0 and ('IQS kernel PM PASS case='+case).encode() in stdout
            rows.append(dict(case=case,repeat=repeat,exit=code,pass_verified=valid))
            print(stem,code,'PASS' if valid else 'FAIL',flush=True)
    after = identity()
    (out/'source-after.json').write_text(json.dumps(after,indent=2))
    (out/'results.json').write_text(json.dumps(rows,indent=2))
    if before != after or not all(row['pass_verified'] for row in rows):
        raise SystemExit(1)

if __name__ == '__main__':
    main()
