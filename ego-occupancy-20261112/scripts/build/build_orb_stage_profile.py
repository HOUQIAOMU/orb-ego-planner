#!/usr/bin/env python3
"""Build an ABI-preserving instrumented core alongside an unchanged built ORB checkout."""
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

def once(text,old,new):
    if text.count(old)!=1:raise RuntimeError('Expected exactly one instrumentation anchor: '+old[:100])
    return text.replace(old,new,1)

def entry(text,signature,stage,pre=''):
    if text.count(signature)!=1:raise RuntimeError('Function signature not unique: '+signature)
    start=text.index(signature);brace=text.index('{',start+len(signature))
    return text[:brace+1]+'\n'+pre+'    stage_profile::Scope orbprof_scope_'+stage.lower()+'(stage_profile::'+stage+');\n'+text[brace+1:]

def instrument_sources(original):
    system=original['System.cc']
    start=system.index('Sophus::SE3f System::TrackRGBD(');end=system.index('Sophus::SE3f System::TrackMonocular(',start)
    part=system[start:end]
    part=entry(part,'Sophus::SE3f System::TrackRGBD(','TOTAL','    stage_profile::Begin(timestamp);\n')
    part=once(part,'    cv::Mat imToFeed = im.clone();','    stage_profile::Scope orbprof_input(stage_profile::INPUT_COPY);\n    cv::Mat imToFeed = im.clone();')
    part=once(part,'    cv::Mat imDepthToFeed = depthmap.clone();','    cv::Mat imDepthToFeed = depthmap.clone();\n    orbprof_input.finish();')
    part=once(part,'        cv::Mat resizedIm;','        stage_profile::Scope orbprof_resize(stage_profile::RESIZE);\n        cv::Mat resizedIm;')
    for lock,name in [('mMutexMode','MODE'),('mMutexReset','RESET')]:
        target='        unique_lock<mutex> lock('+lock+');'
        part=once(part,target,'        stage_profile::Scope orbprof_'+name.lower()+'_wait(stage_profile::'+name+'_LOCK_WAIT);\n'+target+'\n        orbprof_'+name.lower()+'_wait.finish();\n        stage_profile::Scope orbprof_'+name.lower()+'_actions(stage_profile::'+name+'_ACTIONS);')
    target='    unique_lock<mutex> lock2(mMutexState);'
    part=once(part,target,'    stage_profile::Scope orbprof_state_wait(stage_profile::STATE_LOCK_WAIT);\n'+target+'\n    orbprof_state_wait.finish();\n    stage_profile::Scope orbprof_publish(stage_profile::STATE_PUBLISH);')
    system=system[:start]+part+system[end:]

    tracking=original['Tracking.cc']
    start=tracking.index('Sophus::SE3f Tracking::GrabImageRGBD(');end=tracking.index('Sophus::SE3f Tracking::GrabImageMonocular(',start)
    part=tracking[start:end]
    part=entry(part,'Sophus::SE3f Tracking::GrabImageRGBD(','GRAB_RGBD')
    part=once(part,'    mImGray = imRGB;','    stage_profile::Scope orbprof_gray(stage_profile::GRAY_CONVERT);\n    mImGray = imRGB;')
    target='    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)'
    part=once(part,target,'    orbprof_gray.finish();\n    stage_profile::Scope orbprof_depth(stage_profile::DEPTH_CONVERT);\n'+target)
    target='    if (mSensor == System::RGBD)'
    part=once(part,target,'    orbprof_depth.finish();\n    stage_profile::Scope orbprof_frame(stage_profile::FRAME_BUILD);\n'+target)
    part=once(part,'    mCurrentFrame.mNameFile = filename;','    orbprof_frame.finish();\n    mCurrentFrame.mNameFile = filename;')
    tracking=tracking[:start]+part+tracking[end:]
    functions={
        'void Tracking::Track()':'TRACKING',
        'void Tracking::StereoInitialization()':'INITIALIZATION',
        'bool Tracking::TrackReferenceKeyFrame()':'REFERENCE_TRACK',
        'bool Tracking::TrackWithMotionModel()':'MOTION_TRACK',
        'bool Tracking::Relocalization()':'RELOCALIZATION',
        'bool Tracking::TrackLocalMap()':'LOCAL_MAP_TRACK',
        'void Tracking::UpdateLocalMap()':'LOCAL_MAP_UPDATE',
        'void Tracking::SearchLocalPoints()':'LOCAL_POINT_SEARCH',
        'bool Tracking::NeedNewKeyFrame()':'KEYFRAME_DECISION',
        'void Tracking::CreateNewKeyFrame()':'KEYFRAME_CREATE',
        'void Tracking::CreateMapInAtlas()':'NEW_MAP',
        'void Tracking::ResetActiveMap(bool bLocMap)':'ACTIVE_MAP_RESET'}
    for signature,stage in functions.items():tracking=entry(tracking,signature,stage)
    target='    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);'
    tracking=once(tracking,target,'    stage_profile::Scope orbprof_map_wait(stage_profile::MAP_LOCK_WAIT);\n'+target+'\n    orbprof_map_wait.finish();\n    stage_profile::Scope orbprof_map_hold(stage_profile::MAP_LOCK_HOLD);')
    replacements={
        'Optimizer::PoseOptimization(&mCurrentFrame)':'stage_profile::Measure(stage_profile::POSE_OPTIMIZE, [&]() { return Optimizer::PoseOptimization(&mCurrentFrame); })',
        'mpFrameDrawer->Update(this)':'stage_profile::Measure(stage_profile::FRAME_DRAWER_UPDATE, [&]() { mpFrameDrawer->Update(this); })',
        'mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose())':'stage_profile::Measure(stage_profile::MAP_DRAWER_UPDATE, [&]() { mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose()); })'}
    lines=[]
    for line in tracking.splitlines(keepends=True):
        if not line.lstrip().startswith(('//','*')):
            for old,new in replacements.items():line=line.replace(old,new)
        lines.append(line)
    tracking=''.join(lines)
    frame=original['Frame.cc']
    for signature,stage in {
        'void Frame::ExtractORB(':'ORB_EXTRACT',
        'void Frame::UndistortKeyPoints()':'UNDISTORT',
        'void Frame::ComputeStereoFromRGBD(':'DEPTH_ASSOCIATE',
        'void Frame::AssignFeaturesToGrid()':'FEATURE_GRID'}.items():frame=entry(frame,signature,stage)
    return {name:'#include "ORBStageProfile.h"\n'+text for name,text in [('System.cc',system),('Tracking.cc',tracking),('Frame.cc',frame)]}

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def verify_baseline(orb, link):
    """Reject stale CMake objects, including header and build-option changes."""
    build=orb/'build';target=build/'CMakeFiles/ORB_SLAM3.dir'
    configs=[target/'flags.make',target/'build.make',target/'link.txt',build/'CMakeCache.txt']
    objects=[];dependencies={}
    for argument in link:
        if not argument.endswith('.o'):continue
        obj=(build/argument).resolve() if not Path(argument).is_absolute() else Path(argument)
        source=orb/str(obj).split('/ORB_SLAM3.dir/',1)[-1][:-2]
        depfile=Path(str(obj)+'.d')
        if not all(p.is_file() for p in [obj,source,depfile,*configs]):
            raise RuntimeError('Baseline source/object/dependencies missing: '+str(obj))
        deps=shlex.split(depfile.read_text().replace('\\\n',' ').split(':',1)[1])
        paths=[source,*configs]
        paths.extend((Path(p) if Path(p).is_absolute() else build/p).resolve() for p in deps)
        for path in paths:
            if not path.is_file() or path.stat().st_mtime_ns>obj.stat().st_mtime_ns:
                raise RuntimeError('Baseline object stale against dependency: '+str(obj)+' <- '+str(path))
            if str(path) not in dependencies:dependencies[str(path)]=sha(path)
        objects.append(dict(object=str(obj),object_sha256=sha(obj),source=str(source),
                            source_sha256=sha(source),dependency_file_sha256=sha(depfile),
                            dependency_count=len(deps)))
    if not objects:raise RuntimeError('No baseline objects found')
    return objects,dependencies

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--orb-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args();orb=args.orb_root.resolve();output=args.output.resolve()
    build=orb/'build';original={n:(orb/'src'/n).read_text() for n in ['System.cc','Tracking.cc','Frame.cc']}
    changed=instrument_sources(original)
    flags_text=(build/'CMakeFiles/ORB_SLAM3.dir/flags.make').read_text()
    flags={key:shlex.split(value) for key,value in re.findall(r'^(CXX_\w+) = (.*)$',flags_text,re.M)}
    if any('REGISTER_TIMES' in flag for key in flags for flag in flags[key]):raise RuntimeError('REGISTER_TIMES baseline unsupported; public layouts must remain unchanged')
    link=shlex.split((build/'CMakeFiles/ORB_SLAM3.dir/link.txt').read_text())
    if '-shared' not in link:raise RuntimeError('Expected baseline shared-library link command')
    subprocess.run(['git','-C',str(orb),'diff','--exit-code','--','src','include','CMakeLists.txt'],check=True,stdout=subprocess.PIPE)
    objects,dependencies=verify_baseline(orb,link)
    if output.exists() and any(output.iterdir()):raise RuntimeError('Output must be empty/new: '+str(output))
    for directory in ['src','obj','lib']: (output/directory).mkdir(parents=True,exist_ok=True)
    diff=[]
    for name,text in changed.items():
        (output/'src'/name).write_text(text)
        diff.extend(difflib.unified_diff(original[name].splitlines(True),text.splitlines(True),fromfile='baseline/src/'+name,tofile='profile/src/'+name))
    (output/'instrumentation.patch').write_text(''.join(diff))
    profile_dir=Path(__file__).resolve().parent/'orb_stage_profile'
    commands=[];replacements={}
    for name in ['System.cc','Tracking.cc','Frame.cc','ORBStageProfile.cc']:
        source=profile_dir/name if name=='ORBStageProfile.cc' else output/'src'/name
        obj=output/'obj'/(name+'.o')
        command=[link[0],*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-I'+str(profile_dir),'-c',str(source),'-o',str(obj)]
        commands.append(command)
        if name!='ORBStageProfile.cc':replacements[name+'.o']=str(obj)
    output_link=link[:];output_link[output_link.index('-o')+1]=str(output/'lib/libORB_SLAM3.so')
    for i,arg in enumerate(output_link):
        if arg.endswith('.o'):
            name=Path(arg).name
            if name in replacements:output_link[i]=replacements[name]
            elif not Path(arg).is_absolute():output_link[i]=str((build/arg).resolve())
    output_link.append(str(output/'obj/ORBStageProfile.cc.o'));commands.append(output_link)
    manifest=dict(orb_root=str(orb),git_head=subprocess.check_output(['git','-C',str(orb),'rev-parse','HEAD'],text=True).strip(),
        baseline_library_sha256=sha(orb/'lib/libORB_SLAM3.so'),objects=objects,dependencies_sha256=dependencies,commands=commands,abi='unchanged public headers and class layout; REGISTER_TIMES disabled')
    (output/'build-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if args.prepare_only:print('PREPARED '+str(output));return
    for i,command in enumerate(commands):
        print('BUILD_STEP '+str(i+1)+'/'+str(len(commands))+' '+('link' if i==len(commands)-1 else Path(command[-1]).name),flush=True)
        with (output/('build-step-'+str(i+1)+'.log')).open('w') as log:subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
    print('PROFILE_LIBRARY_READY '+str(output/'lib/libORB_SLAM3.so'),flush=True)
if __name__=='__main__':main()
