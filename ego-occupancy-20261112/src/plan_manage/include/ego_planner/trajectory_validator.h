#pragma once

#include <ego_planner/planning_map_view.h>
#include <ego_planner/seed_builder.h>

#include <array>
#include <string>

namespace ego_planner {

enum class ValidationStatus {
  NotRun,
  Valid,
  InvalidInput,
  BoundaryMismatch,
  Collision,
  Unverified,
  DynamicsExceeded,
  GateNotSatisfied,
  Timeout,
  Cancelled
};
const char* toString(ValidationStatus status);
struct ValidationReport {
  ValidationStatus status = ValidationStatus::NotRun;
  std::string reason;
  double boundary_error = 0;
  double speed_bound = 0;
  double acceleration_bound = 0;
  std::size_t visits = 0;
  int collision_span = -1;
  double collision_begin = 0;
  double collision_end = 0;
  int gate_crossings = 0;
};

using BezierSpan = std::array<Vec3, 4>;
BezierSpan bezierSpan(const Bspline& spline, int span);

class TrajectoryValidator {
 public:
  ValidationReport validate(const PlanningMapView& view, const Bspline& spline,
                            const PlanningRequest& request, const PlanningConfig& config,
                            WorkBudget budget) const;
};
}  // namespace ego_planner
