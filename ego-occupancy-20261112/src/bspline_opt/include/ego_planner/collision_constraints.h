#pragma once

#include <ego_planner/astar.h>
#include <ego_planner/seed_builder.h>

#include <string>

namespace ego_planner {

enum class ConstraintStatus {
  NoCollision,
  ConstraintsReady,
  SearchFailed,
  DegenerateConstraint,
  ResourceLimit,
  Timeout,
  Cancelled
};
const char* toString(ConstraintStatus status);

struct ProxyConstraint {
  Eigen::Index control_index = 0;
  Vec3 base_point = Vec3::Zero();
  Vec3 direction = Vec3::UnitX();
};
struct CollisionSegment {
  double begin_time = 0;
  double end_time = 0;
  std::vector<Vec3> astar_path;
};
struct ConstraintResult {
  ConstraintStatus status = ConstraintStatus::NoCollision;
  std::vector<ProxyConstraint> constraints;
  std::vector<CollisionSegment> segments;
  std::size_t astar_calls = 0;
  std::string reason;
};

class CollisionConstraintBuilder {
 public:
  /// forced_spans来自完整曲线验证的碰撞见证，不能由Unverified伪造。
  ConstraintResult build(const PlanningMapView& view, const Bspline& spline,
                         const PlanningConfig& config, WorkBudget budget,
                         const std::vector<int>& forced_spans = {}) const;
};

}  // namespace ego_planner
