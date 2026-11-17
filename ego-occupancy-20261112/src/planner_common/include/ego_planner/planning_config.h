#pragma once

#include <ego_planner/planning_request.h>

namespace ego_planner {

/// 仅为手持实验参数；不能作为真实机体尺寸和飞行性能的证明。
struct PlanningConfig {
  Vec3 half_extent = Vec3::Constant(0.05);
  double nominal_speed = 0.5;
  double max_velocity = 1.0;
  double max_acceleration = 2.0;
  double control_point_spacing = 0.15;
  double roi_margin = 0.5;
  double soft_clearance = 0.05;
  double smoothness_weight = 1.0;
  double collision_weight = 50.0;
  double feasibility_weight = 0.1;
  double fitness_weight = 0.1;
  double boundary_tolerance = 1e-6;
  double gate_tolerance = 1e-6;
  double max_duration = 120.0;
  std::size_t max_control_points = 64;
  std::size_t max_roi_voxels = 500'000;
  std::size_t max_validation_visits = 300'000;
  int max_subdivision_depth = 22;
  int max_iterations = 120;
  int max_restarts = 4;
  int max_time_adjustments = 3;
  bool allow_roi_expansion = true;
};

void validateConfig(const PlanningConfig& config);
void validateRequest(const PlanningRequest& request, const PlanningConfig& config);

}  // namespace ego_planner
