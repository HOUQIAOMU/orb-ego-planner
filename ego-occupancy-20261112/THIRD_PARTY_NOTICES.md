# Algorithm provenance

The probabilistic occupancy, per-frame hit/miss voting and obstacle inflation
design follows EGO-Planner's `plan_env`, checked at commit
`bfda51284c8c1b476043255a8145ef925a3778a5`:
https://github.com/ZJU-FAST-Lab/ego-planner

EGO-Planner is distributed under GNU GPL version 3. New mapping code in this
directory is provided under GNU GPL version 3 or later; see `LICENSE`.
Copyright (c) 2026 contributors to this project. Original EGO-Planner authors
include Xin Zhou, Zhepei Wang, Chao Xu and Fei Gao (Zhejiang University).

The voxel traversal uses the Amanatides–Woo geometric method. This is a fresh
implementation with explicit map-origin offset, continuous ray direction and
bounded indices; it does not copy the original EGO traversal implementation.
