# SDD ledger — plan: docs/superpowers/plans/2026-11-12-ego-current-frame.md

User explicitly requested camera pose and RGB current-frame display, after authorizing native implementation/SSH deployment. Small fully specified display extension; no new approval round needed. Existing no-git ruling remains in effect.

Pre-flight: Task1 supplies CurrentFrameBuffer/cameraPoseForMap/cameraFrustum to Task2. CameraModel is already calibrated; SourceStamp and ReaderStatus are the existing map contract. No public ORB layout or bridge ABI changes.

Root cause verified in source: depth is transformed by the copied T_world_camera, but display receives no pose/image and draws only the fixed origin axes. Last strict real replay had20/20 verified poses and10 map updates. Position chain already exists; presentation chain is absent.

Task1: complete. Missing current_frame.h reproduced feature-absent RED. Implemented owned latest-frame RGB buffer, source/epoch/current-bridge/freshness pose gate, calibrated full rotation/translation geometry. Workstation RGB-D7/7 and display8/8 CTests PASS. Existing36 Python tests PASS (10.18s).

Task2: implemented app/display integration. Reused Pangolin context, yellow frustum and local RGB axes, camera XYZ/tracking/map freshness labels, bottom-right current RGB texture with frame/state header. Pose updates use the same record and image consumed by tracking; RGB remains independent of pose validity. No ORB core changes. ARM build/tests and visible real-sequence replay in progress; one independent review dispatched.

First visible ARM replay: 180/180 verified poses,90 fused, exit0, RSS656.37MiB. Data /home/lckfb/orb-slam3/data/ego-current-frame-replay-20261112-01. Screenshot confirmed yellow frustum and current RGB pane but exposed blue modulation from the preceding OpenGL draw color.

Final review: Important timing issue accepted: clock/status sampled before full-grid traversal can reject a newer RGB frame as future or keep an expired pose LIVE. Add atomic latest-frame/time sample after traversal and obtain fresh source status. Missing sample API RED, then mailbox/frame-age tests GREEN to be recorded below.
Final review regrade: exception-path GL window retention is material to the stated owning-thread lifecycle, so fix in this pass instead of deferring the reviewer's Minor label. Actual desktop regression screenshot-failure retained-window test RED. Rendering-color regression also RED against actual ARM GL: constant BGR image does not survive previous blue draw state. Fix with owning-thread RAII window cleanup and white texture modulation, then verify full suite.

Review declined visual appearance/performance: handled by real ARM replay/screenshots and explicit color regression, not assumed from code. Review declined ORB pose/calibration and map geometry precision: remain separate physical acceptance; this display change cannot establish them.

Final fix verification: workstation8/8 non-GUI contracts PASS; two opt-in actual-desktop contracts SKIP because sandbox cannot access its DISPLAY. Earlier first GUI attempt failed to open X, so test precondition now explicit EGO_TEST_DESKTOP=1, and cleanup must specifically report the post-creation screenshot-write error, not accept startup failure. ARM EGO_TEST_DESKTOP=1 full11/11 CTests PASS (4.20s), both graphics contracts exercised. Frame sampling tests GREEN, rendered color and owning-thread context cleanup RED→GREEN. No deferred review findings.

Final review rulings: exception-window cleanup upgraded to material because source promised owning-thread GL lifetime; fixed and covered. Physical ORB/calibration/map accuracy remain unmeasured, so make no precision improvement claim. Visual/performance evidence collected separately on actual ARM instead of delegated review. No additional core/bridge ABI or ROS changes.

Task2: complete. Final actual camera run20s saved260/260 pairs, verified255/255 processed poses,128 map updates, exit0, RSS710.06MiB. Age p50 155.77ms/p95 258.36ms/max278.91ms, no >500ms updates, no map queue evictions. Strict recording/frame-source verification PASS, exported timestamp rounding0.0sec. Desktop screenshot frame98 shows same XYZ as CSV after 2-decimal rounding (-0.13,-0.01,0.11), Map/Camera LIVE, Tracking OK, yellow frustum and normal-color live RGB. Window leftmost18px cropped by desktop placement; user can drag window; not a map/pose defect. No image editing performed.

Final evidence: logs/ego-current-frame-20261112/{build,live}; ARM data /home/lckfb/orb-slam3/data/ego-current-frame-live-20261112-01. Same31 tool source/header/CMake hashes on host/ARM. Baseline original ORB library hash remains cd0dbad1b2e906c147a7ce499ed1287236d66306f71d929adb218fd8fe68326b; no running ego-rgbd-map after bounded test; swap remained disabled. Source review complete and all material findings fixed. Geometric calibration/pose accuracy remain separate acceptance; camera is color optical origin in SLAM local coordinates, not a calibrated drone body position.
