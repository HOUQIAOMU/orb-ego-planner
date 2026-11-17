"""Reproduce offline acceptance cases using the real C++ planner on the board."""
import argparse
import hashlib
import importlib.util
import json
import pathlib
import subprocess
import struct


def write_free_fixture(path):
    """Public snapshot format; entirely synthetic Free volume, never a live observation."""
    frame=b'synthetic_free_volume'
    dimensions=(25,21,13)
    count=dimensions[0]*dimensions[1]*dimensions[2]
    data=bytearray(b'EGOOCC01')
    data+=struct.pack('<I3Id3d6QdI',1,*dimensions,.1,0.,0.,0.,7,8,9,2,3,4,1.,len(frame))
    data+=frame+struct.pack('<Q',count)+bytes([1])*count+bytes(count)
    checksum=14695981039346656037
    for value in data:checksum=((checksum^value)*1099511628211)&((1<<64)-1)
    data+=struct.pack('<Q',checksum)
    with path.open('xb') as stream:stream.write(data)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ego-root',type=pathlib.Path,required=True)
    parser.add_argument('--output',type=pathlib.Path,required=True)
    parser.add_argument('--real-map',type=pathlib.Path)
    args=parser.parse_args()
    ego=args.ego_root.resolve()
    root=args.output.resolve()
    root.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('fixtures',ego/'tests/support/fixtures.py')
    fixtures=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixtures)
    fixtures.generate(root/'fixtures')
    write_free_fixture(root/'fixtures/free-volume.egovox')
    tasks=root/'tasks';tasks.mkdir()
    centered=(ego/'config/tasks/example_gate.yaml').read_text()
    (tasks/'gate.yaml').write_text(centered)
    (tasks/'rebound.yaml').write_text('start_position: [-0.725, -0.375, 0.025]\n'
                                      'goal_position: [0.725, -0.375, 0.025]\n')
    (tasks/'experiment.yaml').write_text((ego/'config/planning.yaml').read_text()+'\n')
    cases=[('gate',root/'fixtures/open.egovox',tasks/'gate.yaml',tasks/'experiment.yaml',0),
           ('rebound',root/'fixtures/open.egovox',tasks/'rebound.yaml',tasks/'experiment.yaml',0),
           ('closed',root/'fixtures/closed.egovox',tasks/'gate.yaml',tasks/'experiment.yaml',3),
           ('unknown',root/'fixtures/unknown.egovox',tasks/'gate.yaml',tasks/'experiment.yaml',3)]
    (tasks/'offset-gate.yaml').write_text(
        'start_position: [0.25, 0.85, 0.55]\ngoal_position: [2.15, 0.85, 0.55]\n'
        'reference_0: [0.95, 1.4, 0.55]\nreference_1: [1.55, 1.4, 0.55]\n'
        'gate_center: [1.25, 1.4, 0.55]\ngate_normal: [1, 0, 0]\ngate_u: [0, 1, 0]\n'
        'gate_v: [0, 0, 1]\ngate_half_width: 0.2\ngate_half_height: 0.3\n')
    (tasks/'retime.yaml').write_text(
        'start_position: [0.25, 0.85, 0.55]\ngoal_position: [2.15, 0.85, 0.55]\n'
        'start_velocity: [0.1, 0.03, 0]\ngoal_velocity: [0.05, 0, 0]\n'
        'start_acceleration: [0.01, 0, 0]\ngoal_acceleration: [0, 0.005, 0]\n')
    (tasks/'retime-config.yaml').write_text('half_extent: [0, 0, 0]\nmax_acceleration: 0.05\nroi_margin: 1.0\n')
    cases.extend([
        ('offset-gate',root/'fixtures/free-volume.egovox',tasks/'offset-gate.yaml',tasks/'experiment.yaml',0),
        ('retime',root/'fixtures/free-volume.egovox',tasks/'retime.yaml',tasks/'retime-config.yaml',0)])
    if args.real_map:
        (tasks/'real.yaml').write_text('start_position: [-1.025, -0.325, 1.775]\n'
                                      'goal_position: [-0.625, -0.325, 1.775]\n')
        (tasks/'real-config.yaml').write_text('half_extent: [0.1, 0.1, 0.1]\nroi_margin: 0.25\n')
        cases.append(('real',args.real_map,tasks/'real.yaml',tasks/'real-config.yaml',0))
    summary=[]
    binary=ego/'build/planner/bin/ego-plan-snapshot'
    for name,map_path,task,config,expected in cases:
        command=[str(binary),str(map_path),str(task),str(config),str(root/name),'5000']
        process=subprocess.run(command,capture_output=True,text=True,timeout=30)
        if not (root/name).exists():
            raise RuntimeError(f'{name}: output missing: {process.stdout}{process.stderr}')
        (root/name/'command.json').write_text(json.dumps(command,ensure_ascii=False,indent=2)+'\n')
        (root/name/'stdout.log').write_text(process.stdout+process.stderr)
        report=json.loads((root/name/'planning-report.json').read_text())
        summary.append({'case':name,'returncode':process.returncode,'expected':expected,
                        'map':str(map_path),'sha256':hashlib.sha256(map_path.read_bytes()).hexdigest(),
                        'report':report})
        print(name,process.returncode,process.stdout.strip(),flush=True)
        subprocess.run(['python3',str(ego/'scripts/validation/render_trajectory_report.py'),str(root/name),str(map_path)],check=True)
        (root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        if process.returncode!=expected:
            raise RuntimeError(f'{name}: expected exit {expected}, got {process.returncode}')
    text=['# RK3576 P1–P3 offline acceptance','',
          'C++ planner outputs; frozen maps and explicit experiment parameters.',
          'Real-map endpoints are selected known-Free positions, not the live camera pose or a physical gate.','',
          '| Case | Status | Validation | Candidate | A* | Evaluations | Total ms |',
          '|---|---|---|---|---:|---:|---:|']
    for item in summary:
        name=item['case'];r=item['report']
        validation=json.loads((root/name/'validation.json').read_text())
        text.append(f"| {name} | {r['status']} | {validation['status']} | {r['selected_candidate']} | {r['astar_calls']} | {r['evaluations']} | {r['timings_ms']['total']:.3f} |")
    for item in summary:
        text+=['',f"## {item['case']}",'',f"![trajectory]({item['case']}/trajectory-report.png)"]
    (root/'README.md').write_text('\n'.join(text)+'\n')
    print('RESULT_DIR='+str(root))


if __name__=='__main__':
    main()
