"""Visualize saved C++ results only; this script never computes a planned path."""
import argparse
import ast
import csv
import json
import pathlib
import struct

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap


def read_rows(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.array([float(row[key]) for row in rows])
            for key in (rows[0].keys() if rows else [])}


def load_map(path):
    data = pathlib.Path(path).read_bytes()
    if data[:8] != b'EGOOCC01':
        raise ValueError('expected public EGOOCC01 snapshot')
    schema, nx, ny, nz, resolution = struct.unpack_from('<I3Id', data, 8)
    if schema != 1:
        raise ValueError('unsupported snapshot schema')
    origin = np.array(struct.unpack_from('<3d', data, 32))
    frame_length = struct.unpack_from('<I', data, 112)[0]
    offset = 116 + frame_length
    count = struct.unpack_from('<Q', data, offset)[0]
    if count != nx * ny * nz:
        raise ValueError('invalid voxel count')
    states = np.frombuffer(data, dtype=np.uint8, count=count, offset=offset+8)
    return states.reshape(nx, ny, nz), resolution, origin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=pathlib.Path)
    parser.add_argument('map', type=pathlib.Path)
    args = parser.parse_args()
    root = args.result
    report = json.loads((root/'planning-report.json').read_text())
    validation = json.loads((root/'validation.json').read_text())
    reference = read_rows(root/'reference.csv')
    trajectory = read_rows(root/'trajectory.csv')
    states, resolution, origin = load_map(args.map)
    low = np.array(report['roi_min'],dtype=int)
    high = np.array(report['roi_max_exclusive'],dtype=int)
    if np.any(high <= low):
        low = np.zeros(3,dtype=int)
        high = np.array(states.shape)
    z = int(np.clip(round((reference.get('z', np.array([origin[2]]))[0]-origin[2])/resolution-.5),0,states.shape[2]-1))
    fig = plt.figure(figsize=(15,10), layout='constrained')
    xy = fig.add_subplot(2,2,1)
    extent = (origin[0]+low[0]*resolution,origin[0]+high[0]*resolution,
              origin[1]+low[1]*resolution,origin[1]+high[1]*resolution)
    xy.imshow(states[low[0]:high[0],low[1]:high[1],z].T,origin='lower',extent=extent,
              cmap=ListedColormap(['#b8bec5','#ffffff','#263d52']),vmin=0,vmax=2,
              interpolation='nearest',aspect='equal')
    xyz = fig.add_subplot(2,2,2,projection='3d')
    for axis in (xy,xyz):
        for rows,color,label,style in ((reference,'#a7a7a7','quintic reference','--'),
                                       (trajectory,'#1267e9','validated B-spline','-')):
            if rows:
                values = [rows[key] for key in ('xy' if axis is xy else 'xyz')]
                axis.plot(*values,color=color,lw=2,label=label,linestyle=style)
        for i,path in enumerate(sorted((root/'astar-paths').glob('*.csv'))):
            rows=read_rows(path)
            if rows:
                axis.plot(*[rows[key] for key in ('xy' if axis is xy else 'xyz')],
                          color='#ef8a23',alpha=.6,lw=1,label='collision-segment A*' if i==0 else None)
        if reference:
            for index,color,label,marker in ((0,'#009c65','start','o'),(-1,'#c43175','goal','*')):
                axis.scatter(*[reference[key][index] for key in ('xy' if axis is xy else 'xyz')],
                             color=color,label=label,marker=marker,s=60)
        axis.set_xlabel('map X [m]'); axis.set_ylabel('map Y [m]');axis.legend(fontsize=8)
    # 用实际ROI作为3D坐标范围，避免把浮点级Z噪声放大成可见起伏。
    bottom = origin + low*resolution
    top = origin + high*resolution
    xyz.set_xlim(bottom[0],top[0]);xyz.set_ylim(bottom[1],top[1]);xyz.set_zlim(bottom[2],top[2])
    xyz.set_box_aspect(np.maximum(top-bottom,resolution))
    occupied=np.argwhere(states[low[0]:high[0],low[1]:high[1],low[2]:high[2]]==2)
    if len(occupied):
        occupied=occupied[::max(1,len(occupied)//4000)]
        points=origin+(occupied+low+.5)*resolution
        xyz.scatter(*points.T,color='#263d52',s=4,alpha=.2,label='raw occupied voxels')
    gate={}
    for line in (root/'request.yaml').read_text().splitlines():
        line=line.split('#',1)[0]
        if ':' not in line: continue
        key,value=line.split(':',1)
        if key.strip().startswith('gate_'):gate[key.strip()]=ast.literal_eval(value.strip())
    if 'gate_center' in gate:
        center=np.array(gate['gate_center']);u=np.array(gate['gate_u']);v=np.array(gate['gate_v'])
        corners=np.array([center+su*gate['gate_half_width']*u+sv*gate['gate_half_height']*v
                          for su,sv in ((-1,-1),(1,-1),(1,1),(-1,1),(-1,-1))])
        xyz.plot(*corners.T,color='#12a56a',lw=2,label='manual gate')
        xy.plot(corners[:,0],corners[:,1],color='#12a56a',lw=2,label='manual gate')
    xyz.legend(fontsize=8);xy.legend(fontsize=8)
    xyz.set_zlabel('map Z [m]')
    xy.set_title(f'Raw map XY slice z={origin[2]+(z+.5)*resolution:.3f} m (unknown=gray)')
    xyz.set_title('Saved trajectory geometry; not a flight execution')
    for position,keys,limit_key,label in ((3,('vx','vy','vz'),'velocity','speed [m/s]'),
                                         (4,('ax','ay','az'),'acceleration','acceleration [m/s²]')):
        axis=fig.add_subplot(2,2,position)
        if trajectory:
            norm=np.sqrt(sum(trajectory[key]**2 for key in keys))
            axis.plot(trajectory['t'],norm,color='#1267e9',label='sampled norm (visual only)')
        # Hard acceptance uses the C++ derivative convex-hull bounds, not these samples.
        key='speed_bound' if position==3 else 'acceleration_bound'
        axis.axhline(report['limits'][limit_key],color='#d74949',linestyle='--',label='configured limit')
        axis.legend(fontsize=8)
        axis.set_title(f'Conservative C++ bound: {validation[key]:.4f}')
        axis.set_xlabel('trajectory time [s]');axis.set_ylabel(label);axis.grid(alpha=.25)
    fig.suptitle(f"RK3576 offline planner | {report['status']} / {validation['status']}\n"
                 f"A* calls {report['astar_calls']} | evaluations {report['evaluations']} | "
                 f"total {report['timings_ms']['total']:.2f} ms | {report['reason']}",fontsize=11)
    fig.savefig(root/'trajectory-report.png',dpi=150)
    fig.savefig(root/'trajectory-report.svg')
    plt.close(fig)
    print(root/'trajectory-report.png')


if __name__=='__main__':
    main()
