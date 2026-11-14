#!/usr/bin/env python3
"""Verify a completed real mapping run against the original RGB-D frame manifest."""
import argparse
import csv
import json
import math
from pathlib import Path
import struct

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run', type=Path)
parser.add_argument('--source-rgbd', type=Path)
parser.add_argument('--allow-legacy-rounded-timestamps', action='store_true',
                    help='Explicitly tolerate the first experimental build\'s 12-significant-digit pose CSV; never changes the recording')
args = parser.parse_args()
root = args.run
data = args.source_rgbd or root / 'rgbd'
frames = list(csv.DictReader((data / 'frames.csv').open()))
associations = [line.split() for line in (data / 'associations.txt').read_text().splitlines() if line.strip() and not line.startswith('#')]
poses = list(csv.DictReader((root / 'poses.csv').open()))
mapping = list(csv.DictReader((root / 'mapping.csv').open()))
assert len(frames) == len(associations) > 0
previous_color = previous_depth = -math.inf
for i, (frame, association) in enumerate(zip(frames, associations)):
    assert len(association) == 4
    ct, color, dt, depth = association
    ct, dt = float(ct), float(dt)
    assert math.isfinite(ct) and math.isfinite(dt) and ct > previous_color and dt > previous_depth
    assert abs(ct - dt) <= .010
    assert int(frame['index']) == i and abs(ct - float(frame['color_timestamp_sec'])) < 1e-8 and abs(dt - float(frame['depth_timestamp_sec'])) < 1e-8
    assert frame['color_domain'] == frame['depth_domain'] == frames[0]['color_domain']
    for name, bits, kind in [(color, 8, 2), (depth, 16, 0)]:
        with (data / name).open('rb') as image:
            header = image.read(26)
        assert header[:8] == b'\x89PNG\r\n\x1a\n'
        width, height, actual_bits, actual_kind = struct.unpack('>IIBB', header[16:26])
        assert width > 0 and height > 0 and (actual_bits, actual_kind) == (bits, kind)
    previous_color, previous_depth = ct, dt
assert len(list((data / 'rgb').glob('*.png'))) == len(list((data / 'depth').glob('*.png'))) == len(frames)
by_index = {}
previous = -1
maximum_rounding = 0.
for pose in poses:
    index = int(pose['capture_index'])
    assert previous < index < len(frames)
    previous = index
    by_index[index] = pose
    actual = float(frames[index]['color_timestamp_sec'])
    difference = abs(float(pose['timestamp_sec']) - actual)
    maximum_rounding = max(maximum_rounding, difference)
    tolerance = 1e-8
    if args.allow_legacy_rounded_timestamps:
        tolerance += .5 * 10 ** (math.floor(math.log10(max(abs(actual), 1))) - 11)
    assert difference <= tolerance, ('pose CSV timestamp lost precision', index, difference, tolerance)
    if pose['pose_frame_verified'] == '1':
        assert pose['state'] == '2' and pose['captured'] == '1'
for row in mapping:
    if row['accepted'] != '1':
        continue
    pose = by_index[int(row['capture_index'])]
    assert pose['pose_frame_verified'] == '1'
    assert pose['orb_frame'] == row['orb_frame'] and pose['map_token'] == row['source_map'] and pose['revision'] == row['source_revision']
result = dict(recorded_pairs=len(frames), processed=len(poses), accepted_updates=sum(r['accepted'] == '1' for r in mapping),
              png_headers='all checked for dimensions/bit depth/color type', timestamps_and_domains='passed',
              frame_and_source_pairing='passed', legacy_rounded_pose_timestamps=args.allow_legacy_rounded_timestamps,
              maximum_pose_csv_rounding_sec=maximum_rounding,
              recorded_fps=(len(frames) - 1) / (previous_color - float(frames[0]['color_timestamp_sec'])))
(root / 'recording-verification.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
