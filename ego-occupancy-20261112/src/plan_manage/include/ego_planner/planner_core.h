#pragma once

#include <ego_planner/rebound_optimizer.h>
#include <ego_planner/trajectory_validator.h>

namespace ego_planner {

enum class PlanStatus {
  Success,
  InvalidInput,
  StartBlocked,
  GoalBlocked,
  ConstraintFailure,
  OptimizerFailure,
  ValidationFailure,
  NoPath,
  ResourceLimit,
  Timeout,
  Cancelled
};
const char* toString(PlanStatus status);
enum class CandidateSource { None, Optimized, SeedFallback };
const char* toString(CandidateSource source);
struct PlanningTimings {
  double seed_ms = 0;
  double view_ms = 0;
  double constraints_ms = 0;
  double optimization_ms = 0;
  double validation_ms = 0;
  double total_ms = 0;
};
struct PlanResult {
  PlanStatus status = PlanStatus::InvalidInput;
  std::string reason;
  std::optional<Bspline> trajectory;
  CandidateSource selected_candidate = CandidateSource::None;
  std::optional<Bspline> initial_spline;
  std::optional<ReferenceTrajectory> reference;
  ego_occupancy::Metadata planned_on;
  Roi roi;
  std::vector<ProxyConstraint> constraints;
  std::vector<CollisionSegment> collision_segments;
  ValidationReport validation;
  OptimizerStatus optimizer_status = OptimizerStatus::NumericalFailure;
  int solver_code = 0;
  std::size_t astar_calls = 0;
  std::size_t evaluations = 0;
  int iterations = 0;
  double smoothness_cost = 0;
  double collision_cost = 0;
  double feasibility_cost = 0;
  double fitness_cost = 0;
  int roi_attempts = 0;
  int restarts = 0;
  int time_adjustments = 0;
  double initial_cost = 0;
  double final_cost = 0;
  PlanningTimings timings;
};

class PlannerCore {
 public:
  /// 同步、可取消、无跨任务状态。结果不持有snapshot或PlanningMapView。
  PlanResult plan(std::shared_ptr<const ego_occupancy::MapSnapshot> snapshot,
                  const PlanningRequest& request, const PlanningConfig& config,
                  WorkBudget budget) const;
};
}  // namespace ego_planner
