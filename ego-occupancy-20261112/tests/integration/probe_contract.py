import csv
import json
import pathlib
import subprocess
import sys
import tempfile
from fixtures import generate,START,GOAL
binary=sys.argv[1]
def run(*args):
    return subprocess.run([binary,*map(str,args)],capture_output=True,text=True)
assert run("--help").returncode==0
with tempfile.TemporaryDirectory(prefix="planning-probe-") as temporary:
    root=pathlib.Path(temporary)
    generate(root/"fixtures")
    args=[*START,*GOAL,.1,2000]
    output=root/"open-result"
    done=run(root/"fixtures/open.egovox",output,*args)
    assert done.returncode==0,done.stdout+done.stderr
    report=json.loads((output/"report.json").read_text())
    assert report["status"]=="Success" and report["mode"]=="OfflineValidation"
    assert report["source"]==[7,8,9] and report["epoch"]==2 and report["sequence"]==3
    assert report["trajectory_optimized"] is False
    path=list(csv.DictReader((output/"path.csv").open()))
    assert len(path)>2
    for axis,value in zip("xyz",START): assert abs(float(path[0][axis])-value)<1e-10
    for axis,value in zip("xyz",GOAL): assert abs(float(path[-1][axis])-value)<1e-10
    assert run(root/"fixtures/open.egovox",output,*args).returncode==2
    for kind in ("closed","unknown"):
        result=root/(kind+"-result")
        done=run(root/("fixtures/"+kind+".egovox"),result,*args)
        assert done.returncode==3,done.stdout+done.stderr
        assert json.loads((result/"report.json").read_text())["status"]=="NoPath"
        assert list(csv.DictReader((result/"path.csv").open()))==[]
    assert run(root/"fixtures/open.egovox",root/"invalid","nan",*args[1:]).returncode==2
    assert not (root/"invalid").exists()
    assert run(root/"fixtures/open.egovox",root/"suffix","0junk",*args[1:]).returncode==2
print("PASS probe: snapshot, endpoints, versions, no-path, no-overwrite, invalid CLI")
