import csv
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

verifier = Path(__file__).resolve().parents[1] / "verify_run.py"


def rows(path, header, data):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(data)


def make_run(root, sampled, depth_offset=0):
    rgbd = root / "rgbd"
    rgbd.mkdir(parents=True)
    (root / "summary.yml").write_text(
        f"mode: live\ncaptured: 8.\nrecorded: {len(sampled)}.\nrecord_skipped_by_rate: 6.\n"
    )
    rows(root / "poses.csv",
         ("capture_index", "orb_frame", "timestamp_sec", "depth_timestamp_sec",
          "pose_frame_verified", "state", "captured", "map_token", "revision"),
         [(i, j, t, t + offset, 1, 2, 1, 11, 0)
          for j, (i, t, offset) in enumerate([(0, 10, 0), (4, 10.4, 0), (7, 10.7, depth_offset)])])
    rows(root / "mapping.csv", ("accepted", "capture_index", "orb_frame", "source_map", "source_revision"),
         [(1, 4, 1, 11, 0), (1, 7, 2, 11, 0)])
    if not sampled:
        return
    for subdir in ("rgb", "depth"):
        (rgbd / subdir).mkdir()
    associations = []
    frames = []
    for index, stamp in enumerate(sampled):
        name = f"{index:06}.png"
        for subdir, bits, kind in (("rgb", 8, 2), ("depth", 16, 0)):
            header = b"\x89PNG\r\n\x1a\n" + bytes(8) + struct.pack(">IIBB", 2, 2, bits, kind)
            (rgbd / subdir / name).write_bytes(header)
        associations.append(f"{stamp} rgb/{name} {stamp} depth/{name}\n")
        frames.append((index, stamp, stamp, 1, 1))
    (rgbd / "associations.txt").write_text("".join(associations))
    rows(rgbd / "frames.csv", ("index", "color_timestamp_sec", "depth_timestamp_sec", "color_domain", "depth_domain"),
         frames)


with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    for name, stamps, offset, expected in (("sampled", [10, 10.7], 0, 0),
                                           ("disabled", [], 0, 0),
                                           ("wrong-depth", [10, 10.7], .003, 1)):
        run = root / name
        make_run(run, stamps, offset)
        result = subprocess.run([sys.executable, str(verifier), str(run)], capture_output=True, text=True)
        assert result.returncode == expected, result.stderr
