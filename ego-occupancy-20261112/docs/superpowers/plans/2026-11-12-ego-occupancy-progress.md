# SDD ledger — plan: docs/superpowers/plans/2026-11-12-ego-occupancy.md

User approved the design, supplied SSH access, requested implementation and display of key core commands on 2026-11-12.

Ruling: Execute inline in current workspace; no usable .git exists — user requested implementation here, so do not initialize a new repository or require another approval — changes are recorded by files, commands and this ledger rather than commits.
Ruling: User's explicit start instruction authorizes the implementation plan without an additional plan-approval round — preserve the approved design and proceed — material scope changes would still be reported.

Pre-flight: Tasks 1→2 share Config/RayObservation/MapSnapshot; Tasks 3→4 share frame-bound bridge record and invalidation generation; Tasks 2+4→5 share CLI and versioned snapshot IO. Interfaces are defined in their owner tasks before consumers.

Baseline: 36 existing Python tests PASS (2026-11-12), log /tmp/ego-occupancy-baseline-tests.log. Host Eigen 3.4.0 / OpenCV 4.6.0. SSH connected; board aarch64 Ubuntu 24.04.4, Eigen 3.4.0/OpenCV 4.6.0/librealsense 2.56.5, 3.2GiB available/no swap. D435i present on USB3. Existing board ORB has prior build.sh/logs changes; leave them intact.

Task 1: complete. Initial configure failed because the implementation was absent (RED); subsequent Release build and voxel_contract passed all 6 behavioral groups. Full core and immutable snapshot tests live in tools/ego-occupancy/tests/map_test.cpp.

Task 2: complete. Depth/IO missing-source RED then Release build and all 3 CTests PASS. Synthetic opening demo produced 512000 voxels; center Free/uninflated/passable, rim Occupied. Saved full grid reload inspected correctly. Data: data/ego-occupancy-synthetic-20261112-01.

Task 3: in progress. Bridge lifecycle tests written first; configuration confirmed absent bridge implementation (RED). Board baseline original library SHA256 cd0dbad1b2e906c147a7ce499ed1287236d66306f71d929adb218fd8fe68326b. Board Tracking/LoopClosing hashes match local, System differs, so transformation reads actual source instead of copying local source.

Task 3 build: independent board core built by compiling System/Tracking/LoopClosing and reusing verified unchanged objects; original library hash unchanged. Temporary existing /swapfile enabled only when Tracking compilation approached memory exhaustion, then disabled successfully before application/camera tests. No permanent system configuration changed. Real frame validation still pending.

Task 4: source implemented. Workstation 6 core/depth/worker/CLI CTests PASS; core-only build (no OpenCV/SLAM/RealSense/Pangolin) 4/4 PASS. Initial full ARM app build + 7 CTests PASS before review fixes.

Final review: fresh reviewer found one Important display-construction race and four material minors. Ruling: treat all four as contract defects worth fixing in this same pass — valid small grids must round-trip, --check must use writer validation, parser must agree with ORB settings version, and invalid maps must be visibly marked — leaving them would mislead callers or displayed-map interpretation.
Final review RED: deterministic legal thread schedule reproduced lost display startup error; tiny snapshot load and missing camera version tests failed locally; actual ARM executable accepted invalid probabilities during --check. Fixes applied; workstation display-enabled suite now 7/7 PASS, ARM rebuild/retest pending. UI stale-state behavior to be checked in visible runtime.

Ruling: sample map config uses size [8,4,8] rather than [8,8,4] — RGB-D-only initial optical Z points forward, not up, so this keeps the same 2,048,000 voxels while extending forward depth from 2m to 4m — final scene/vehicle clearance parameters remain unvalidated.

Final fix-pass follow-up: ARM display test initially intercepted pre-main OpenBLAS workers and timed out. GDB identified blas_thread_init waiting inside its worker; arm the forced schedule only immediately before the actual display construction and force an unsupported Pangolin backend. ARM full suite then 8/8 PASS; workstation display suite 7/7 PASS. Existing 36 Python regressions PASS.

Additional snapshot lifetime audit: writer-owned shared_ptr storage would allow a reader's weak_ptr to resurrect a slot during later reuse. Regression weak_snapshot_cannot_resurrect_recycled_storage reproduced RED. Switch to fresh publication ownership blocks over 3 bounded backing slots; expired publication weak_ptrs cannot be revived. Verify GREEN and ARM redeployment below.

Runtime 1: recorded-sequence replay, 60/60 verified poses, 30 fused observations, 3,473 occupied voxels, 2,048,000 total voxels; 588.64MiB RSS peak; exit 0. Real independent bridge loaded, no ROS libraries.
Runtime 2 (user-assisted, partial opening, stride2): 860/860 RGB-D pairs saved, 842/842 processed poses verified, 416 updates, 10 map-queue evictions, 16 SLAM evictions, no SDK frame gaps, 735.54MiB peak; exit0. Mapping age p50 341.01ms, p95 513.57ms, max624.69ms; 25/416 updates completed older than 500ms. User reported gray invalid display; matches freshness threshold. Camera-image and pose/source indices verified for every fused update.
Runtime 3 (user-assisted stride4): 412/412 pairs saved, 409 processed, 359 verified, 45 RECENTLY_LOST/3 LOST/2 initialization, 181 updates, no map evictions, 704.15MiB peak; exit0. Source revisions 1/3/5/7 and reset logs observed, final epoch60; invalid poses not fused. Age p50 131.45ms, p95 211.02ms, max238.09ms; no fused update older than500ms. Fusion p50 22.39ms versus106.71ms in the stride2 run. Different handheld motions: do not attribute tracking-loss difference or geometric precision to stride alone. No measured aperture-dimension accuracy yet.

Runtime validation found a logging defect: poses.csv used 12 significant digits for epoch-scale sensor seconds, rounding by up to about5ms. Internal binding checks and binary map doubles were unaffected. Strict verify_run.py reproduced RED on preserved original run; change pose CSV to max_digits10 and verify a fresh exact-timestamp replay. Preserve old CSV files as evidence and require explicit --allow-legacy-rounded-timestamps for their export-precision limitation.


Final verification: weak-publication ownership regression GREEN on workstation and ARM. ARM application rebuilt with final sources; final CTest 8/8 PASS (1.90s), core-only workstation rebuild 4/4 PASS. Display-enabled workstation 7/7 PASS; original Python tests 36/36 PASS. Shell syntax and Python compiler checks PASS. Independent review findings all resolved; no material review item deferred.

Tasks 3–5: implementation and experiment execution complete. Final strict replay of the first live recording processed 20/20 verified poses and fused10 updates; verify_run.py checked all860 source image pairs and every accepted update's frame/map/revision identity. Maximum pose CSV rounding0.0sec, no legacy tolerance. Final map 2,048,000 voxels: unknown1,997,833/free48,403/occupied1,764, inflated40,287; saved and reloaded on ARM.

Final ARM synthetic opening: 512,000 voxels; loaded center state Free, inflationfalse, blockedfalse; loaded rim Occupied, inflationtrue, blockedtrue. New data: /home/lckfb/orb-slam3/data/ego-occupancy-synthetic-arm-20261112-01. Final replay: /home/lckfb/orb-slam3/data/ego-occupancy-final-replay-20261112-01; run log /home/lckfb/orb-slam3/logs/ego-occupancy-20261112-165935-145324610.

Final environment check: baseline ORB library SHA256 still cd0dbad1b2e906c147a7ce499ed1287236d66306f71d929adb218fd8fe68326b. /proc/swaps empty, no running ego-rgbd-map processes. pgrep exit1 means no process matched, not a failed CTest. Original camera/SLAM baseline preserved.

Ruling: keep default stride2 unchanged and document explicit --stride4 as the next RK experiment setting — it improved measured load/age but reduces sampling density and may miss thin geometry. Do not increase freshness timeout to hide gray states. No acceptance claim for metric opening accuracy: partial-opening handheld runs completed, but user reported unclear geometry, pose loss occurred, and no ruler/known-dimension reference was recorded. Planning/flight remain out of scope.

Deliverables and results documented in docs/ego-occupancy-2026-11-12.md; complete query grid, compact runtime records and build evidence retained under logs/ego-occupancy-20261112. Raw real RGB-D remains on board. No commit created because the root .git is unusable and read-only.
