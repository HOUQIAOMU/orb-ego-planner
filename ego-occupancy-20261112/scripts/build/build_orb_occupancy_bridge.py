#!/usr/bin/env python3
"""Build an isolated ABI-preserving ORB RGB-D observation bridge; never overwrite baseline."""
import argparse
import difflib
import json
from pathlib import Path
import re
import shlex
import subprocess
from build_orb_stage_profile import once, verify_baseline, sha


def entry(text, signature, inserted):
    if text.count(signature) != 1:
        raise RuntimeError('Function signature not unique: ' + signature)
    start = text.index(signature)
    brace = text.index('{', start + len(signature))
    return text[:brace + 1] + '\n' + inserted + '\n' + text[brace + 1:]


def instrument(original):
    system = entry(original['System.cc'], 'Sophus::SE3f System::TrackRGBD(',
                   '    ego_pose_bridge::BeginFrame(timestamp);')
    tracking = original['Tracking.cc']
    anchor = '    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);'
    tracking = once(tracking, anchor, anchor + '''
    // Destruct BEFORE the existing map lock: pose/state/map are one protected copy.
    ego_pose_bridge::CaptureGuard ego_capture([&]() {
        const bool valid = mCurrentFrame.isSet() && mpAtlas->GetCurrentMap() == pCurrentMap;
        Eigen::Matrix4f Twc = Eigen::Matrix4f::Identity();
        if(valid) Twc = mCurrentFrame.GetPose().inverse().matrix();
        ego_pose_bridge::Capture(mCurrentFrame.mnId, mCurrentFrame.mTimeStamp,
                                 pCurrentMap, Twc.data(), mState, valid);
    });''')
    for signature, reason in [
        ('void Tracking::CreateMapInAtlas()', 'new_map'),
        ('void Tracking::Reset(bool bLocMap)', 'reset_all'),
        ('void Tracking::ResetActiveMap(bool bLocMap)', 'reset_active')]:
        tracking = entry(tracking, signature,
                         '    ego_pose_bridge::CorrectionScope ego_change("' + reason + '");')
    loop = original['LoopClosing.cc']
    for signature, reason in [
        ('void LoopClosing::CorrectLoop()', 'loop_correction'),
        ('void LoopClosing::MergeLocal()', 'map_merge'),
        ('void LoopClosing::MergeLocal2()', 'map_merge_inertial')]:
        loop = entry(loop, signature,
                     '    ego_pose_bridge::CorrectionScope ego_change("' + reason + '");')
    # Suspend only GBA application, not the potentially long optimization itself.
    anchor = '            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);'
    loop = once(loop, anchor, '            ego_pose_bridge::CorrectionScope ego_gba("global_ba_apply");\n' + anchor)
    return {name: '#include "pose_bridge.h"\n' + text for name, text in
            [('System.cc', system), ('Tracking.cc', tracking), ('LoopClosing.cc', loop)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--orb-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bridge-include', type=Path, required=True)
    parser.add_argument('--bridge-library', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    orb, output = args.orb_root.resolve(), args.output.resolve()
    inc, lib = args.bridge_include.resolve(), args.bridge_library.resolve()
    if not (inc / 'pose_bridge.h').is_file() or not lib.is_file():
        raise RuntimeError('Build ego_pose_bridge first; header/library missing')
    build = orb / 'build'
    original = {name: (orb / 'src' / name).read_text() for name in ['System.cc', 'Tracking.cc', 'LoopClosing.cc']}
    changed = instrument(original)
    flags = {key: shlex.split(value) for key, value in re.findall(
        r'^(CXX_\w+) = (.*)$', (build / 'CMakeFiles/ORB_SLAM3.dir/flags.make').read_text(), re.M)}
    if any('REGISTER_TIMES' in f for values in flags.values() for f in values):
        raise RuntimeError('REGISTER_TIMES baseline unsupported')
    link = shlex.split((build / 'CMakeFiles/ORB_SLAM3.dir/link.txt').read_text())
    subprocess.run(['git', '-C', str(orb), 'diff', '--exit-code', '--', 'src', 'include', 'CMakeLists.txt'], check=True)
    objects, dependencies = verify_baseline(orb, link)
    if output.exists() and any(output.iterdir()):
        raise RuntimeError('Output must be new/empty: ' + str(output))
    for name in ['src', 'obj', 'lib']:
        (output / name).mkdir(parents=True, exist_ok=True)
    patch, commands, replacements = [], [], {}
    for name, text in changed.items():
        source, obj = output / 'src' / name, output / 'obj' / (name + '.o')
        source.write_text(text)
        patch.extend(difflib.unified_diff(original[name].splitlines(True), text.splitlines(True),
                                        fromfile='baseline/src/' + name, tofile='bridge/src/' + name))
        commands.append([link[0], *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'],
                         *flags['CXX_FLAGS'], '-I' + str(inc), '-c', str(source), '-o', str(obj)])
        replacements[name + '.o'] = str(obj)
    output_link = link[:]
    output_link[output_link.index('-o') + 1] = str(output / 'lib/libORB_SLAM3.so')
    for i, arg in enumerate(output_link):
        if arg.endswith('.o'):
            output_link[i] = replacements.get(Path(arg).name, str((build / arg).resolve()))
    output_link.extend([str(lib), '-Wl,-rpath,' + str(lib.parent)])
    commands.append(output_link)
    (output / 'bridge.patch').write_text(''.join(patch))
    manifest = dict(orb_root=str(orb), baseline_library_sha256=sha(orb / 'lib/libORB_SLAM3.so'),
                    git_head=subprocess.check_output(['git', '-C', str(orb), 'rev-parse', 'HEAD'], text=True).strip(),
                    objects=objects, dependencies_sha256=dependencies, commands=commands,
                    bridge_header_sha256=sha(inc / 'pose_bridge.h'), bridge_library_sha256=sha(lib),
                    abi='original public headers/class layout unchanged; separate numeric bridge API')
    (output / 'build-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    if args.prepare_only:
        print('BRIDGE_PREPARED ' + str(output)); return
    for i, command in enumerate(commands):
        print('BUILD_STEP ' + str(i + 1) + '/' + str(len(commands)) + ' ' + shlex.join(command), flush=True)
        with (output / ('build-step-' + str(i + 1) + '.log')).open('w') as log:
            subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
    if sha(orb / 'lib/libORB_SLAM3.so') != manifest['baseline_library_sha256']:
        raise RuntimeError('Baseline library changed during isolated build')
    print('BRIDGE_LIBRARY_READY ' + str(output / 'lib/libORB_SLAM3.so'), flush=True)


if __name__ == '__main__':
    main()
