"""Relocation checks must reject stale ABI/object inputs before touching the live core."""
from pathlib import Path
import hashlib,json,subprocess,sys,tempfile,unittest,runpy
import os
TOOL=Path(__file__).resolve().parents[2]/'scripts/build/relink_orb_bridge.py'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
class RelinkInputs(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.project=Path(self.tmp.name);self.ego=self.project/'ego-occupancy-20261112';orb=self.project/'ORB_SLAM3'
        for d in [self.ego/'core/lib',self.ego/'core/obj',self.ego/'core/src',self.ego/'src/orb_bridge/include',self.ego/'build/release/lib',orb/'lib',orb/'build']:d.mkdir(parents=True,exist_ok=True)
        for p in [orb/'lib/libORB_SLAM3.so',orb/'build/baseline.o',self.ego/'core/obj/System.cc.o',self.ego/'core/src/System.cc',self.ego/'src/orb_bridge/include/pose_bridge.h',self.ego/'build/release/lib/libego_pose_bridge.so']:
            p.write_bytes(b'fixture '+p.name.encode())
        self.live=self.ego/'core/lib/libORB_SLAM3.so';self.live.write_bytes(b'preserve live core')
        old=self.project/'old-ego'
        manifest={'orb_root':str(orb),'baseline_library_sha256':digest(orb/'lib/libORB_SLAM3.so'),
                  'dependencies_sha256':{},'objects':[{'object':str(orb/'build/baseline.o'),'object_sha256':digest(orb/'build/baseline.o')}],
                  'bridge_header_sha256':digest(self.ego/'src/orb_bridge/include/pose_bridge.h'),
                  'commands':[['/usr/bin/c++','-o',str(old/'core/lib/libORB_SLAM3.so'),str(old/'core/obj/System.cc.o'),str(orb/'build/baseline.o'),str(old/'build/libego_pose_bridge.so'),'-Wl,-rpath,'+str(old/'build')]]}
        origin=self.ego/'core/build-manifest.json';origin.write_text(json.dumps(manifest))
        migration={'old_ego_root':str(old),'original_manifest_sha256':digest(origin),
                   'patched_objects':{'System.cc.o':digest(self.ego/'core/obj/System.cc.o')},
                   'patched_sources':{'System.cc':digest(self.ego/'core/src/System.cc')}}
        (self.ego/'core/relocation-input.json').write_text(json.dumps(migration))
    def check(self):
        r=subprocess.run([sys.executable,str(TOOL),'--ego-root',str(self.ego),'--bridge-library',str(self.ego/'build/release/lib/libego_pose_bridge.so'),'--check'],capture_output=True,text=True)
        self.assertEqual(self.live.read_bytes(),b'preserve live core')
        return r
    def test_valid_inputs_check_without_replacing_existing_core(self):
        r=self.check();self.assertEqual(r.returncode,0,r.stderr)
    def test_modified_original_object_rejected(self):
        (self.project/'ORB_SLAM3/build/baseline.o').write_bytes(b'changed object')
        r=self.check();self.assertNotEqual(r.returncode,0);self.assertIn('Hash mismatch',r.stderr)
    def test_modified_patch_object_rejected(self):
        (self.ego/'core/obj/System.cc.o').write_bytes(b'changed patch object')
        self.assertNotEqual(self.check().returncode,0)
    def test_loader_origin_dotdot_path_resolves_to_new_bridge(self):
        validate=runpy.run_path(str(TOOL))['validate_linkage']
        bridge=self.ego/'build/release/lib/libego_pose_bridge.so'
        loader_path=self.ego/'core/lib/../../build/release/lib/libego_pose_bridge.so'
        validate('libego_pose_bridge.so => '+str(loader_path)+' (0x1234)\n',bridge)
        with self.assertRaises(RuntimeError):
            validate('libego_pose_bridge.so => '+str(self.project/'wrong/libego_pose_bridge.so')+' (0x1234)\n',bridge)
        with self.assertRaises(RuntimeError):validate('libego_pose_bridge.so => not found\n',bridge)
    def test_changed_bridge_header_rejected(self):
        (self.ego/'src/orb_bridge/include/pose_bridge.h').write_bytes(b'ABI changed')
        self.assertNotEqual(self.check().returncode,0)
    def test_explicit_bridge_rewrites_link_and_rpath(self):
        bridge=self.ego/'build/release/lib/libego_pose_bridge.so'
        command,target,actual,_=runpy.run_path(str(TOOL))['prepare'](self.ego,bridge)
        self.assertIn(str(bridge),command)
        self.assertIn('-Wl,-rpath,$ORIGIN/../../build/release/lib',command)
        self.assertEqual(actual,bridge)
        self.assertFalse(any('old-ego' in word for word in command))
    def test_missing_explicit_bridge_rejected(self):
        (self.ego/'build/release/lib/libego_pose_bridge.so').unlink()
        r=self.check();self.assertNotEqual(r.returncode,0)
if __name__=='__main__':unittest.main()
