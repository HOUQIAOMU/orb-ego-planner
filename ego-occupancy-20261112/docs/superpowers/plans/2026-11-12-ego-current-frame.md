# EGO Current Frame Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Show the verified camera pose and current RGB frame in the existing ROS-free occupancy display.
**Architecture:** One owned latest-frame mailbox; source/epoch/freshness-gated pose queries; Pangolin frustum and RGB texture fed by the existing tracking loop.
**Tech Stack:** Existing C++17, OpenCV, Eigen and Pangolin; unchanged ORB bridge ABI.
**Spec:** docs/superpowers/specs/2026-11-12-ego-current-frame-design.md

## Global Constraints

Preserve original ORB core, baseline launchers and recordings. No ROS, new camera pipeline, planner or additional ORB hooks. Root .git remains unusable; record files and evidence instead of commits. Continue the previously authorized native execution and SSH deployment; display key commands.

## Review Focus

Never render a stale or mismatched coordinate pose as current. RGB must continue through tracking loss. Do not block the tracking loop on rendering. GL context and texture destruction stay in the display thread. Source corrections must invalidate poses even without another image.

### Task 1: Owned frame mailbox and camera geometry

Files: add include/ego_occupancy/current_frame.h, src/current_frame.cpp, tests/current_frame_test.cpp; update CMakeLists.txt.
Interfaces: CurrentFrameBuffer::publish/latest; CurrentFrame contains owned BGR, record, capture index, receive time and verification; cameraPoseForMap returns optional T_world_camera; cameraFrustum returns five world points using calibrated optical axes.

- [x] Write/run failing ownership, newest-only, invalid-pose RGB, age/source/epoch and full-rotation geometry tests.
- [x] Implement the data path in the RGB-D library, independent of GL; run all RGB-D CTests.

### Task 2: App and display integration, ARM verification

Files: update display.h/cpp, apps/rgbd_map.cpp, display_startup_test.cpp and usage documentation.
Interfaces: OccupancyDisplay receives CameraModel and pose-age limit; publishCurrentFrame forwards the bound frame to its mailbox. Existing map query contract remains unchanged.

- [x] Integrate per-processed-frame RGB/pose update, calibrated frustum, camera XYZ/status and RGB pane; preserve RGB on tracking failure.
- [x] Run local display-enabled suite, deploy new app sources and build ARM with -j1; run complete ARM CTest and existing Python regressions.
- [x] Run bounded visible replay of a previous real recording; inspect screenshot and pose/map indices, record timings and baseline hash.
- [x] Obtain one independent final review and fix material findings with reproducing tests; document final evidence and limitations.
