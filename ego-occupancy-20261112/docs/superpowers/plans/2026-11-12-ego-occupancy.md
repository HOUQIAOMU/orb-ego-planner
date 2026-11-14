# EGO Occupancy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build and validate a ROS-free RGB-D voxel mapper with immutable in-process planning queries on the workstation and RK3576.

**Architecture:** A single writer fuses calibrated depth rays into probabilistic dense voxels and rebuilds inflation. A bounded snapshot pool exposes immutable queries; a separate source/epoch controller invalidates stale observations and results. An isolated ORB bridge supplies coherent pose/frame/map metadata to replay and live adapters.

**Tech Stack:** C++17, CMake, Eigen3; optional OpenCV, Pangolin, librealsense and existing ORB-SLAM3. Python3 for deployment/build preparation.

**Spec:** `docs/superpowers/specs/2026-11-12-ego-occupancy-design.md`

## Global Constraints

- No ROS, IMU, ESDF, planning optimizer, controller, or replacement SLAM system.
- Preserve existing sources, binaries, launchers, recordings and GUI baselines; create isolated experimental builds.
- C++17, fixed configurable dense grid; Unknown/Free/Occupied/Outside distinct; unknown and outside blocked by default.
- Immutable bounded 3-slot snapshots; no writer blocking on readers; coordinate invalidation independent of publication.
- Sensor times remain separate from steady-clock freshness; frame and pose stay paired.
- Show key actual shell commands to the user; never print SSH passwords or private keys.
- Root workspace has no usable git metadata: no worktree, initialization, staging or fake commits. Keep a durable progress record instead.

## Review Focus

- Malformed/untrusted snapshot dimensions and truncated files must fail before excessive allocation.
- Ray paths crossing negative coordinates and voxel corners must not leave holes or manufacture boundary obstacles.
- Slow readers holding all slots must not block reset or let old coordinate epochs become usable.
- ORB early returns, loop correction, merging and reset must not leak a prior frame's valid pose.
- Aligned depth distortion, non-finite values and changed device calibration must not silently distort geometry.

### Task 1: Voxel core and planning query contract

**Files:** create `tools/ego-occupancy/CMakeLists.txt`, `include/ego_occupancy/map.h`, `src/map.cpp`, `tests/map_test.cpp`, `THIRD_PARTY_NOTICES.md`.

**Interfaces:** produce `Config`, `SourceStamp`, `RayObservation`, `OccupancyMapper::integrate`, `reset`, `invalidate`, `MapReader::acquire/isUsable`, `MapSnapshot::query/isBlocked/worldToIndex/indexToWorld/forEach`, `EgoGridMapAdapter`.

- [x] Add failing behavioral tests for known axis/diagonal rays, unknown/free/hit thresholds, endpoints and clipped boundaries; configure/build and observe missing implementation failure.
- [x] Implement validated geometry, frame-aggregated log-odds, observation flags and reversible box inflation.
- [x] Add failing snapshot ownership, pool exhaustion, reset/source rejection and freshness tests; implement immutable 3-slot publication and control metadata.
- [x] Run `cmake -S tools/ego-occupancy -B build/ego-occupancy -DCMAKE_BUILD_TYPE=Release`, `cmake --build build/ego-occupancy -j2`, `ctest --test-dir build/ego-occupancy --output-on-failure`; expected all core contracts pass.

### Task 2: Depth projection, portable snapshot IO and inspectable demo

**Files:** create `include/ego_occupancy/depth.h`, `include/ego_occupancy/io.h`, `src/depth.cpp`, `src/io.cpp`, `apps/map_demo.cpp`, depth/IO tests, `config/ego-occupancy.yaml`.

**Interfaces:** consume Task 1; produce calibrated ray LUT/projector, strict full-grid save/load, PLY export and a synthetic opening demo with query output.

- [x] Write/run failing tests for metric Z16/float input, distortion-supported LUT, invalid depth, same-frame pose association and depth/Euclidean range difference.
- [x] Implement adapters with explicit model/units validation; no hole filling or fake far returns.
- [x] Write/run failing roundtrip, bad size/version/truncation/nonfinite metadata tests; implement versioned bounded snapshot files and occupied/inflated PLY.
- [x] Run depth/IO contracts and synthetic aperture demo; expected raw opening and reduced inflated opening agree with known fixture.

### Task 3: Verified isolated ORB observation bridge

**Files:** create `bridge/` record API/implementation and source preparation script, tests; modify only generated copies of actual dependency sources.

**Interfaces:** produce a frame-bound numeric pose/state/source record and invalidation generation; consume actual board source/object manifests, no Map pointers in consumer observations.

- [x] Audit actual source hashes, build flags and correction lock paths, then define exact hooks without changing class layout.
- [x] Write/run failing bridge lifecycle tests (begin/early return, reset, correction, old-frame reuse and concurrent invalidation).
- [x] Build independent ORB core by recompiling affected units and reusing checked unchanged objects; record hashes, flags and original library hash.
- [x] Validate real short replay has one coherent bridge record per processed frame and expected failures are rejected; original core hash unchanged.

### Task 4: Replay/live mapper and optional display

**Files:** create standalone mapping applications/adapters in `tools/ego-occupancy/apps/`, launcher/build helpers and integration tests.

**Interfaces:** consume existing owned RGB-D frames, Tasks 1–3; produce maps, diagnostics, snapshot files and Pangolin occupancy display with direct voxel queries.

- [x] Write/run behavioral CLI/preflight/replay tests; implement existing-sequence replay with bridge identity checks and final map output.
- [x] Implement optional display with raw/inflated voxels and camera pose; keep graphics context in its owning thread.
- [x] Write/run live bounded-worker/stop/calibration rejection tests; integrate a small experimental entry with D435i and the independent core, preserving existing baseline launchers.
- [x] Verify valid final snapshot queries after save/load and no-ROS linkage; log source epoch, tracking rejects, timings, queue drops, age and memory.

### Task 5: ARM64 deployment and measured validation

**Files:** add `docs/ego-occupancy-2026-11-12.md`; retain build/run evidence under ignored `logs/` and data under `data/`.

- [x] Copy only new tools/scripts/config to an isolated board directory; display and run board CMake/build commands with `-j1`.
- [x] Run ARM64 CTest, synthetic demo, recorded-sequence replay and snapshot query roundtrip.
- [x] Run a bounded real camera smoke test, then offer a user-assisted handheld opening measurement; distinguish static validation from geometry/flight acceptance.
- [x] Run existing regression suite, fresh independent code review, fix important findings with regression tests, then re-run affected suites.
- [x] Document exact successful commands, measured limits, artifact paths and remaining hardware acceptance; no claim of planning or flight completion.


## Delivery status — 2026-11-12

Implementation, isolated ARM deployment, review fixes, automated checks, synthetic opening, recorded replay and two bounded handheld runs are complete. Hardware geometry acceptance remains open: user reported unclear rim display and gray invalid periods; a known-dimension opening and repeatable trajectory have not yet been measured. This checklist records execution of the experiments and interfaces, not acceptance of flight precision or completion of planning.
