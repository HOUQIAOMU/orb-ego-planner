"""Render saved C++ probe results; no planning algorithm lives in this script."""
import csv
import json
import pathlib
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle

def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))

def render(root):
    root=pathlib.Path(root)
    scene=json.loads((root/"fixtures/scene.json").read_text())
    nx,ny,nz=scene["dimensions"]
    res=scene["resolution"]
    ox,oy,oz=scene["origin"]
    fig,axes=plt.subplots(2,3,figsize=(16,9),layout="constrained")
    reports=[]
    path=[]
    for column,kind in enumerate(("open","closed","unknown")):
        report=json.loads((root/kind/"report.json").read_text())
        reports.append((kind,report))
        grid=np.ones((ny,nx))
        for y in range(ny):
            grid[y,20]=(1 if kind=="open" else 0 if kind=="unknown" else 2) if 10<=y<20 else 2
        ax=axes[0,column]
        ax.imshow(grid,origin="lower",extent=(ox,ox+nx*res,oy,oy+ny*res),
                  vmin=0,vmax=2,cmap=ListedColormap(["#97a1ad","#ffffff","#263d52"]),
                  interpolation="nearest",aspect="equal")
        points=rows(root/kind/"path.csv")
        if points:
            coordinates=np.array([[float(p[a]) for a in "xyz"] for p in points])
            ax.plot(coordinates[:,0],coordinates[:,1],color="#ed622f",lw=2,label="A* polyline")
            path=coordinates
        for name,color,marker in (("start","#009a66","o"),("goal","#ba2671","*")):
            p=scene[name];ax.scatter(p[0],p[1],c=color,marker=marker,s=55,label=name)
        half=scene["half_extent"][0]
        ax.add_patch(Rectangle((scene["start"][0]-half,scene["start"][1]-half),
                               2*half,2*half,fill=False,edgecolor="#009a66",lw=1.4))
        ax.set_title(f'{kind}: {report["status"]}\n'
                     f'prepare {report["prepare_ms"]:.2f} ms / search {report["astar_ms"]:.2f} ms')
        ax.set_xlabel("map X [m]");ax.set_ylabel("map Y [m]")
        ax.legend(loc="upper right",fontsize=8)
    math=rows(root/"open/math.csv")
    time=np.array([float(row["t"]) for row in math])
    for axis,key,label in ((axes[1,0],"x","X position [m]"),
                           (axes[1,1],"speed","speed norm [m/s]"),
                           (axes[1,2],"acceleration","acceleration norm [m/s^2]")):
        axis.plot(time,[float(row["poly_"+key]) for row in math],label="quintic")
        axis.plot(time,[float(row["spline_"+key]) for row in math],"--",label="B-spline least-squares fit")
        axis.set_xlabel("time [s]");axis.set_ylabel(label);axis.grid(alpha=.25);axis.legend(fontsize=8)
    fig.suptitle("RK3576 | standalone A* + mathematics | 5 cm voxels, 0.2 m experiment box\n"
                 "Top: offline collision search. Bottom: independent math example, NOT an optimized obstacle trajectory.")
    fig.savefig(root/"report.png",dpi=160)
    fig.savefig(root/"report.svg")
    plt.close(fig)
    fig=plt.figure(figsize=(10,7))
    ax=fig.add_subplot(projection="3d")
    occupied=[(.025,oy+(y+.5)*res,oz+(z+.5)*res)
              for y in range(ny) for z in range(nz)
              if not (10<=y<20 and 5<=z<15)]
    wall=np.array(occupied)
    ax.scatter(*wall.T,c="#506d85",s=10,alpha=.5,label="occupied wall voxels")
    if len(path):
        ax.plot(*path.T,color="#ed622f",lw=3,label="A* polyline")
        ax.scatter(*path[0],c="#009a66",s=60,label="start")
        ax.scatter(*path[-1],c="#ba2671",s=60,label="goal")
    ax.set_xlabel("map X [m]");ax.set_ylabel("map Y [m]");ax.set_zlabel("map Z [m]")
    ax.set_box_aspect((nx,ny,nz));ax.legend()
    ax.set_title("Open window: raw occupied voxels and verified A* edges\n"
                 "Map axes are coordinates, not a gravity-aligned flight frame.")
    fig.savefig(root/"geometry-3d.png",dpi=160);plt.close(fig)
    text=["# RK3576 independent A* and math results","",
          "Synthetic offline scenes, 5 cm resolution, half extent 0.10 m.",
          "A* output is a polyline; rebound optimization and flight are not included.","",
          "| Scene | Status | Prepare ms | A* ms | Expanded | Length m |",
          "|---|---|---:|---:|---:|---:|"]
    for kind,p in reports:
        text.append(f'| {kind} | {p["status"]} | {p["prepare_ms"]:.3f} | {p["astar_ms"]:.3f} | {p["expanded"]} | {p["length_m"]:.4f} |')
    text+=["","![Summary](report.png)","","![3D path](geometry-3d.png)","",
           "Each scene contains report.json, path.csv and math.csv. CTest logs/XML are separate evidence."]
    (root/"README.md").write_text("\n".join(text)+"\n")
    print(root/"report.png")
    print(root/"geometry-3d.png")

if __name__=="__main__":
    render(sys.argv[1])
