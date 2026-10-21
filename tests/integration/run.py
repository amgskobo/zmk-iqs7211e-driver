#!/usr/bin/env python3
"""Owner API/DT compile fixture on an explicitly resolved unchanged supported core."""
import argparse, hashlib, json, os, shutil, subprocess, tempfile, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',required=True,type=Path)
    parser.add_argument('--core',required=True,choices=('stock-zmk','dya'))
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(); workspace=args.workspace.resolve(); output=args.output
    if not output.is_absolute() or output.exists() or output.is_symlink():
        parser.error('output must be a fresh absolute path')
    output.mkdir(parents=True,exist_ok=False)
    lock=json.loads((ROOT/'tests/integration/dependencies.json').read_text())
    def project(name):
        return Path(subprocess.check_output(['west','list',name,'-f','{abspath}'],cwd=workspace,text=True).strip())
    zmk=project('zmk');zephyr=project('zephyr')
    selected=[]
    listing=subprocess.check_output(['west','list','-f','{name}|{abspath}'],cwd=workspace,text=True)
    modules=[]
    for line in listing.splitlines():
        name,path=line.split('|',1);path=Path(path)
        if name in ('zmk','zephyr','manifest') or name.startswith('zmk-'):continue
        if (path/'zephyr/module.yml').exists() or (path/'zephyr/CMakeLists.txt').exists():modules.append(str(path))
    for name,path,expected in [('zmk',zmk,lock[args.core]),('zephyr',zephyr,lock['zephyr'])]:
        head=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
        if head!=expected or subprocess.check_output(['git','-C',str(path),'status','--porcelain'],text=True).strip():
            raise SystemExit(f'{name}: expected clean {expected}, got {head}')
        selected.append(dict(name=name,path=str(path),revision=head))
    (output/'dependencies.json').write_text(json.dumps(selected,indent=2))
    spec=json.loads((ROOT/'tests/integration/contract.json').read_text())
    with tempfile.TemporaryDirectory(prefix=ROOT.name+'-interface-') as folder:
        stage=Path(folder)/'module'
        shutil.copytree(ROOT,stage,ignore=shutil.ignore_patterns('.git','node_modules','artifacts','dist','.build','build','__pycache__','.claude'),symlinks=True)
        hashes={p.relative_to(stage).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in stage.rglob('*') if p.is_file() and not p.is_symlink()}
        (output/'source-inputs.json').write_text(json.dumps(hashes,indent=2))
        results=[]
        env=dict(os.environ,ZEPHYR_BASE=str(zephyr),Zephyr_DIR=str(zephyr/'share/zephyr-package/cmake'))
        for variant in spec['variants']:
            destination=output/variant['name'];destination.mkdir()
            build=Path(folder)/variant['name']
            command=['west','build','-p=always','-o=-j4','-s',str(zmk/'app'),'-d',str(build),'-b','xiao_ble//zmk','--',
                     '-DSHIELD='+spec['shield'],'-DZMK_CONFIG='+str(stage/'tests/integration/config'),
                     '-DZEPHYR_MODULES='+';'.join(modules),'-DZMK_EXTRA_MODULES='+str(stage)]
            if variant.get('extra_conf'):command.append('-DEXTRA_CONF_FILE='+str(stage/variant['extra_conf']))
            if variant.get('extra_overlay'):command.append('-DEXTRA_DTC_OVERLAY_FILE='+str(stage/variant['extra_overlay']))
            start=time.monotonic()
            with (destination/'build.log').open('wb') as log:result=subprocess.run(command,cwd=workspace,env=env,stdout=log,stderr=subprocess.STDOUT)
            row=dict(variant=variant['name'],command=command,exit=result.returncode,seconds=time.monotonic()-start)
            results.append(row)
            for rel in ['zephyr/.config','zephyr/zephyr.dts','zephyr/zmk.elf','zephyr/zmk.uf2','compile_commands.json','CMakeCache.txt','zephyr_modules.txt']:
                p=build/rel
                if p.is_file():shutil.copyfile(p,destination/p.name)
            (output/'results.json').write_text(json.dumps(results,indent=2))
            if result.returncode:
                print((destination/'build.log').read_text(errors='replace')[-8000:]);raise SystemExit(result.returncode)
            config=(build/'zephyr/.config').read_text();commands=json.loads((build/'compile_commands.json').read_text())
            for line in spec['config']+variant.get('config',[]):
                if line not in config.splitlines():raise AssertionError('Missing config '+line)
            compiled={str(Path(item['file']).resolve()) for item in commands}
            for relative in spec['sources']:
                if str((stage/relative).resolve()) not in compiled:raise AssertionError('Not compiled '+relative)
            if not (build/'zephyr/zmk.uf2').is_file():raise AssertionError('No firmware UF2')
            row['status']='PASS';(output/'results.json').write_text(json.dumps(results,indent=2))
            print(args.core,variant['name'],'PASS (API/DT compilation only)',flush=True)
    for item in selected:
        if subprocess.check_output(['git','-C',item['path'],'status','--porcelain'],text=True).strip():raise AssertionError('Dependency changed')
if __name__=='__main__':main()
