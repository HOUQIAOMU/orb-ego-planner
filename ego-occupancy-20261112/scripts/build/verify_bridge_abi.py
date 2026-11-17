#!/usr/bin/env python3
"""Compare the current bridge header with the exact header used for the preserved ORB objects."""
from pathlib import Path
import argparse,hashlib,json,subprocess,tempfile

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ego-root',type=Path,default=Path(__file__).resolve().parents[2]);a=p.parse_args();r=a.ego_root.resolve()
    m=json.loads((r/'core/build-manifest.json').read_text());original=r/'core/provenance/original/pose_bridge.h';current=r/'src/orb_bridge/include/pose_bridge.h';probe=r/'scripts/build/bridge_abi_probe.cpp'
    if sha(original)!=m['bridge_header_sha256']:raise RuntimeError('Original ABI reference does not match historical manifest')
    outputs=[]
    with tempfile.TemporaryDirectory(prefix='ego-bridge-abi-') as tmp:
        for i,header in enumerate([original,current]):
            binary=Path(tmp)/str(i)
            subprocess.run(['c++','-std=c++11','-I'+str(header.parent),str(probe),'-o',str(binary)],check=True)
            outputs.append(subprocess.check_output([str(binary)],text=True))
    if outputs[0]!=outputs[1]:raise RuntimeError('Bridge ABI probe differs; preserved ORB objects cannot be reused')
    report={'original_header_sha256':sha(original),'current_header_sha256':sha(current),'probe_sha256':sha(probe),'compiler':subprocess.check_output(['c++','-dumpmachine'],text=True).strip(),'probe_output':outputs[0]}
    (r/'core/bridge-abi-compatibility.json').write_text(json.dumps(report,indent=2)+'\n');print('BRIDGE_ABI_VERIFIED '+report['compiler']+' '+outputs[0].splitlines()[0])
if __name__=='__main__':main()
