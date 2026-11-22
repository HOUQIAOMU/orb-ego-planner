#pragma once

#include <ego_planner/bspline.h>
#include <ego_planner/planning_config.h>
#include <ego_planner/polynomial_traj.h>

namespace ego_planner {

class ReferenceTrajectory {
 public:
  static ReferenceTrajectory MinimumJerk(const PlanningRequest& request, double duration);
  Vec3 evaluate(double time, int derivative_order = 0) const;
  double duration() const;
  const std::vector<PolynomialTraj>& segments() const { return segments_; }

 private:
  std::vector<PolynomialTraj> segments_;
};

struct Seed {
  ReferenceTrajectory reference;
  Bspline spline;
  double interval;
};

/// 均匀三次样条的精确端点；首尾三列由p/v/a唯一确定。
void enforceBoundary(Eigen::MatrixXd& points, double interval, const BoundaryState& start,
                     const BoundaryState& goal);
double uniformInterval(const Bspline& spline);

class SeedBuilder {
 public:
  Seed build(const PlanningRequest& request, const PlanningConfig& config,
             double duration_scale = 1.0) const;
};

}  // namespace ego_planner
