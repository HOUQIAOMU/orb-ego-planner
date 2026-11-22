# L-BFGS source

Copied from local EGO-Planner commit bfda51284c8c1b476043255a8145ef925a3778a5,
`src/planner/bspline_opt/include/bspline_opt/lbfgs.hpp`; preserve project GPL-3.0
and the original header text. No ROS dependency.

Port patches: check allocations and release partially allocated workspace;
add cancellation callbacks before initial evaluation and each outer iteration.
The original More-Thuente line search already invokes progress each trial.
The wrapper checks shared WorkBudget in objective and progress and rejects
Timeout/Cancelled even when a solver stopping branch reports convergence.
Constraints remain immutable during one optimizer call.
