"""Exercise the real offline binary, file contract and failure safety."""
import csv
import json
import pathlib
import subprocess
import sys
import tempfile
from fixtures import generate

binary = sys.argv[1]
def run(*args):
    return subprocess.run([binary, *map(str,args)], capture_output=True, text=True, timeout=30)
assert run('--help').returncode == 0
with tempfile.TemporaryDirectory(prefix='planner-cli-') as temp:
    root=pathlib.Path(temp)
    generate(root/'maps')
    config=root/'planning.yaml'
    config.write_text('half_extent: [0.05, 0.05, 0.05]\nroi_margin: 1.0\n')
    task=root/'task.yaml'
    task.write_text('start_position: [-0.725, 0.025, 0.025]\ngoal_position: [0.725, 0.025, 0.025]\n'
        'reference_0: [-0.25, 0.025, 0.025]\nreference_1: [0.25, 0.025, 0.025]\n'
        'gate_center: [0.025, 0, 0]\ngate_normal: [1, 0, 0]\ngate_u: [0, 1, 0]\n'
        'gate_v: [0, 0, 1]\ngate_half_width: 0.25\ngate_half_height: 0.25\n')
    out=root/'open'
    done=run(root/'maps/open.egovox', task,config,out,5000)
    assert done.returncode==0, done.stdout+done.stderr
    report=json.loads((out/'planning-report.json').read_text())
    validation=json.loads((out/'validation.json').read_text())
    assert report['status']=='Success' and validation['status']=='Valid'
    assert report['source']==[7,8,9] and report['mode']=='OfflineFrozenSnapshot'
    assert validation['gate_crossings']==1
    assert report['iterations'] > 0 and report['selected_candidate'] in ('Optimized','SeedFallback')
    assert {'smoothness','collision','feasibility','fitness'} <= report['cost_terms_raw'].keys()
    assert (out/'bspline.json').exists() and (out/'constraints.json').exists()
    rows=list(csv.DictReader((out/'trajectory.csv').open()))
    assert rows and {'t','x','y','z','vx','vy','vz','ax','ay','az'}<=rows[0].keys()
    assert run(root/'maps/open.egovox',task,config,out,5000).returncode==2
    for kind in ('closed','unknown'):
        out=root/kind
        done=run(root/f'maps/{kind}.egovox',task,config,out,5000)
        assert done.returncode==3,done.stdout+done.stderr
        assert json.loads((out/'planning-report.json').read_text())['status']!='Success'
        assert json.loads((out/'validation.json').read_text())['status']=='NotRun'
        assert not (out/'bspline.json').exists()
        assert list(csv.DictReader((out/'trajectory.csv').open()))==[]
    config.write_text('max_velocity: nan\n')
    assert run(root/'maps/open.egovox',task,config,root/'bad',5000).returncode==2
    assert not (root/'bad').exists()
    config.write_text('misspelled_threshold: 1\n')
    assert run(root/'maps/open.egovox',task,config,root/'bad-key',5000).returncode==2
    config.write_text('roi_margin: 1\nroi_margin: 2\n')
    assert run(root/'maps/open.egovox',task,config,root/'duplicate',5000).returncode==2
    assert run(root/'maps/open.egovox',task,config,root/'bad-time','1junk').returncode==2
print('PASS planner CLI: gate, structured reports, no-overwrite, closed/unknown failure, strict input')
