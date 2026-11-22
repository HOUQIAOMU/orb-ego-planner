# Planner foundations source and limits

- Upstream reference: local EGO-Planner checkout, commit bfda51284c8c1b476043255a8145ef925a3778a5.
- Reference files: bspline_opt/src/uniform_bspline.cpp, path_searching/src/dyn_a_star.cpp,
  traj_utils/src/polynomial_traj.cpp. Original license: GNU GPL version 3, preserved in ../LICENSE.
- Bspline De Boor / derivative / least-squares parameterization is adapted from the above mathematics.
  The old misleading Nx3 comment is corrected to 3xN; dependencies on ROS are removed.
- A* is reimplemented with immutable queue entries, bounded contiguous workspace, explicit failures,
  exact endpoint preservation and conservative 26-neighbor edge checks.
- PolynomialTraj retains the normalized-time single quintic API. The 2026-11-15 SeedBuilder adds
  bounded C2 multi-segment minimum-JERK references using a decomposition solve. Upstream minSnapTraj
  has a third-derivative Q matrix; its name is not used as a claim of minimum fourth-derivative snap.
- Parameterize preserves upstream least-squares semantics; arbitrary boundary samples need not be
  interpolated exactly. SeedBuilder imposes exact uniform-cubic endpoint control points, and the
  independent validator checks p/v/a after optimization and time adjustment.
- normBounds reports derivative-control-point convex-hull bounds in 3D Euclidean norm.
- C++17, Eigen; the core has no ROS, OpenCV, RealSense, ORB or Pangolin dependency.
- View masks are conservative over complete candidate voxels, not exact continuous free-space geometry.
  Unknown/Outside are blocked; raw Occupied and Unknown are dilated together. Existing inflated bytes
  are not dilated again. ROI boundary is treated as blocked.
- WorkBudget is a cooperative deadline/cancellation bound, not hard real-time scheduling.
  Snapshot loading and output/plot IO are outside the reported map-preparation + search budget.
- Report plots use Python/Matplotlib only after the C++ executable has finished. They do not plan.

- Review fix: a nonzero body uses a CLOSED candidate-voxel envelope, including merely touching
  Unknown/Occupied/Outside cells. With integral half_extent/resolution, radius is floor(ratio)+1.
  This is intentionally more conservative than checking only voxel centers. A zero-size point
  keeps the raw center mask, but arbitrary real endpoints use closed-contact neighboring-cell checks.
- Known minor limitation: some repeated interior knot vectors can evaluate position but their
  derivative construction is rejected. The current uniform-knot path is unaffected; general
  repeated-knot derivative support is deferred, and such inputs must not be used in this milestone.

## Optimizer integration (2026-11-15)

- Rebound smoothness, collision proxy, per-axis feasibility and anisotropic refine fitness costs
  are adapted from bspline_opt/src/bspline_optimizer.cpp at the same upstream commit (GPL-3.0).
- Constraints are immutable within one L-BFGS call. Rebound restarts, A* and full validation share
  a cooperative WorkBudget. See third_party/SOURCE.md for exact solver safety patches.
- A sampled curve collision between otherwise Free control points generates a witness-based
  proxy plane propagated to supporting movable controls, generalizing EGO's short-segment case.
- Collision discovery currently samples the complete curve; the final Bezier convex-hull/AABB
  validator is authoritative. Discrete discovery is never used as a safety certificate.
- Unlike the upstream first-two-thirds check, every curve span including fixed-end support is
  validated. Conservative floating-point padding and bounded subdivision reject uncertainty.
- Uniform cubic first/last three control points stay fixed; moving-boundary retiming rebuilds
  them before refine. No simple time scaling is treated as preserving nonzero measured p/v/a.
- Gate semantics, conservative whole-curve checking, explicit request/result statuses, bounded ROI,
  pure event FSM and offline report IO are new implementation components, not copied ROS wrappers.
- P4 online worker, latest-map commit gating and live display layers have not been deployed.

- Final review fixes: explicit near-uniform knots are not accepted as a uniform certificate;
  constructor-known dt is preserved through time scaling, and Bezier segments are obtained from
  the SAME spline's actual knot interval, endpoint positions and endpoint derivatives.
- Fully verified feasible seeds may be retained after optimization loses gate feasibility. The
  fallback is revalidated within the same remaining budget, is explicitly labeled SeedFallback,
  and is never used after cancellation/timeout.
- FSM source/epoch reset also invalidates state. State/task events bind the current source;
  task request_id is strictly increasing within one FSM lifetime to reject delayed commands.
