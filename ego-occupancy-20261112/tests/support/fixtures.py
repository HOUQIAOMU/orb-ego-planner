"""Synthetic offline fixtures; never modify a live map."""
import json
import pathlib
import struct
DIMS=(40,30,20)
RESOLUTION=.05
ORIGIN=(-1.,-.75,-.5)
START=(-.725,-.375,.025)
GOAL=(.725,-.375,.025)

def classify(kind,x,y,z):
    if x!=20: return 1
    if 10<=y<20 and 5<=z<15: return {"open":1,"closed":2,"unknown":0}[kind]
    return 2

def write_fixture(path,kind):
    frame=b"synthetic_optical"
    data=bytearray(b"EGOOCC01")
    data+=struct.pack("<I3Id3d6QdI",1,*DIMS,RESOLUTION,*ORIGIN,7,8,9,2,3,4,1.,len(frame))
    data+=frame
    count=DIMS[0]*DIMS[1]*DIMS[2]
    data+=struct.pack("<Q",count)
    data+=bytes(classify(kind,x,y,z) for x in range(DIMS[0])
                for y in range(DIMS[1]) for z in range(DIMS[2]))
    data+=bytes(count)
    checksum=14695981039346656037
    for value in data: checksum=((checksum^value)*1099511628211)&((1<<64)-1)
    data+=struct.pack("<Q",checksum)
    with pathlib.Path(path).open("xb") as stream: stream.write(data)

def generate(directory):
    directory=pathlib.Path(directory)
    directory.mkdir(parents=True,exist_ok=False)
    for kind in ("open","closed","unknown"): write_fixture(directory/(kind+".egovox"),kind)
    (directory/"scene.json").write_text(json.dumps({
        "dimensions":DIMS,"resolution":RESOLUTION,"origin":ORIGIN,
        "start":START,"goal":GOAL,"half_extent":[.1]*3,
        "wall_x_index":20,"opening_y_indices":[10,20],"opening_z_indices":[5,15],
        "description":"Synthetic fixed map; non-wall cells are known Free."
    },indent=2))
if __name__=="__main__":
    import sys
    generate(sys.argv[1])
