#!/usr/bin/env python3
"""Relink the existing verified ORB objects after layout migration; never rebuild baseline."""
import argparse,hashlib,json,os,subprocess
from pathlib import Path

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def validate_linkage(linkage,bridge):
    if 'not found' in linkage:raise RuntimeError('Relinked core has an unresolved dependency')
    paths=[line.split('=>',1)[1].strip().rsplit(' (',1)[0] for line in linkage.splitlines()
           if line.strip().startswith('libego_pose_bridge.so =>')]
    if len(paths)!=1 or Path(paths[0]).resolve()!=Path(bridge).resolve():
        raise RuntimeError('Relinked core does not resolve the new bridge library')

def prepare(ego):
    ego=Path(ego).resolve();project=ego.parent
    origin=ego/'core/build-manifest.json';migration=ego/'core/relocation-input.json'
    m=json.loads(origin.read_text());v=json.loads(migration.read_text())
    def require(p,digest):
        if sha(p)!=digest:raise RuntimeError('Hash mismatch; refusing relink: '+str(p))
    require(origin,v['original_manifest_sha256'])
    orb=project/'ORB_SLAM3'
    if str(orb)!=m['orb_root']:raise RuntimeError('Original ORB root moved; requires a separate baseline migration')
    require(orb/'lib/libORB_SLAM3.so',m['baseline_library_sha256'])
    for p,h in m['dependencies_sha256'].items():require(Path(p),h)
    for entry in m['objects']:require(Path(entry['object']),entry['object_sha256'])
    for name,h in v['patched_objects'].items():require(ego/'core/obj'/name,h)
    for name,h in v['patched_sources'].items():require(ego/'core/src'/name,h)
    header=ego/'bridge/pose_bridge.h'
    if sha(header)!=m['bridge_header_sha256']:
        compatibility=ego/'core/bridge-abi-compatibility.json'
        if not compatibility.is_file():raise RuntimeError('Hash mismatch; verify current bridge ABI before relink')
        c=json.loads(compatibility.read_text())
        if c['original_header_sha256']!=m['bridge_header_sha256'] or c['current_header_sha256']!=sha(header):
            raise RuntimeError('Hash mismatch; bridge ABI evidence is stale')
        require(ego/'core/provenance/original/pose_bridge.h',m['bridge_header_sha256'])
        require(ego/'scripts/bridge_abi_probe.cpp',c['probe_sha256'])
    bridge=ego/'build/libego_pose_bridge.so'
    if not bridge.is_file():raise RuntimeError('Build ego_pose_bridge before relinking')
    command=[];old=v['old_ego_root']
    for arg in m['commands'][-1]:
        if arg.startswith(old+'/'):arg=str(ego)+arg[len(old):]
        if arg.startswith('-Wl,-rpath,'):
            arg='-Wl,-rpath,'+':'.join('$ORIGIN/../../build' if p.startswith(old+'/') else p for p in arg[len('-Wl,-rpath,'):].split(':'))
        command.append(arg)
    # Validate object path resolution and output before invoking the compiler.
    for arg in command:
        if arg.endswith('.o') and not Path(arg).is_file():raise RuntimeError('Missing verified object '+arg)
    target=ego/'core/lib/libORB_SLAM3.so'
    if command[command.index('-o')+1]!=str(target):raise RuntimeError('Unexpected relink output')
    return command,target,bridge,m

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ego-root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--check',action='store_true');a=p.parse_args()
    command,target,bridge,m=prepare(a.ego_root)
    if a.check:print('RELINK_INPUTS_VERIFIED');return
    temporary=target.with_name('.libORB_SLAM3.relink.tmp.so')
    if temporary.exists():raise RuntimeError('Stale relink output exists; inspect '+str(temporary))
    command[command.index('-o')+1]=str(temporary)
    try:
        subprocess.run(command,cwd=m['orb_root']+'/build',check=True)
        linkage=subprocess.check_output(['ldd',str(temporary)],text=True)
        validate_linkage(linkage,bridge)
        os.replace(temporary,target)
    finally:
        if temporary.exists():temporary.unlink()
    report={'command':command,'baseline_library_sha256':m['baseline_library_sha256'],'bridge_library_sha256':sha(bridge),'relocated_core_sha256':sha(target),'linkage':linkage}
    (target.parent.parent/'relocation-manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print('RELINKED_CORE '+str(target))
if __name__=='__main__':main()
