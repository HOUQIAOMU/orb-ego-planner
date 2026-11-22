"""Real executable preflight: no camera, viewer, output side effects or ORB startup."""
import binascii
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import zlib

binary = Path(sys.argv[1])


def png(path, depth=False):
    def chunk(name, payload):
        return struct.pack('>I', len(payload)) + name + payload + struct.pack('>I', binascii.crc32(name + payload) & 0xffffffff)
    row = b'\x00' + (b'\x03\xe8' * 3 if depth else b'\x80\x90\xa0' * 3)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 3, 1, 16 if depth else 8, 0 if depth else 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(row)) + chunk(b'IEND', b''))


with tempfile.TemporaryDirectory(prefix='ego-app-test-') as temporary:
    root = Path(temporary)
    (root / 'vocabulary').write_text('preflight does not load vocabulary\n')
    camera = root / 'camera.yaml'
    camera.write_text('%YAML:1.0\nFile.version: "1.0"\nCamera.type: "Rectified"\nCamera.width: 3\nCamera.height: 1\nCamera.fps: 15\nCamera.RGB: 0\nCamera1.fx: 2.\nCamera1.fy: 2.\nCamera1.cx: 1.\nCamera1.cy: 0.\nRGBD.DepthMapFactor: 1000.\n')
    map_config = root / 'map.yaml'
    map_config.write_text('%YAML:1.0\nsize_m: [2.,2.,2.]\ninflation_m: [0.,0.,0.]\nresolution_m: 0.1\nmin_ray_m: 0.1\nmax_ray_m: 4.5\nmax_age_sec: 0.5\np_hit: 0.7\np_miss: 0.35\np_min: 0.12\np_max: 0.97\np_free: 0.35\np_occ: 0.8\nframe_id: "test"\n')
    for name in ['rgb.png', 'depth.png']:
        png(root / name, name.startswith('depth'))
    association = root / 'associations.txt'
    association.write_text('1.0 rgb.png 1.0 depth.png\n1.1 rgb.png 1.1 depth.png\n')
    output = root / 'output'
    args = [str(binary), '--vocabulary', str(root / 'vocabulary'), '--camera', str(camera), '--map-config', str(map_config), '--output', str(output), '--dataset', str(root), '--check', '--no-viewer']
    result = subprocess.run(args, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    assert not output.exists(), 'check created output'
    good_map = map_config.read_text()
    map_config.write_text(good_map.replace('p_max: 0.97', 'p_max: 0.4'))
    result = subprocess.run(args, capture_output=True, text=True, timeout=15)
    assert result.returncode == 2, 'invalid probability passed preflight: ' + result.stdout + result.stderr
    assert not output.exists()
    map_config.write_text(good_map)
    association.write_text('1.0 ../outside.png 1.0 depth.png\n1.1 rgb.png 1.1 depth.png\n')
    result = subprocess.run(args, capture_output=True, text=True, timeout=15)
    assert result.returncode == 2, result.stdout + result.stderr
    assert not output.exists()
    result = subprocess.run([str(binary), '--help'], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0
print('preflight valid input, path traversal rejection and no side effects passed')
