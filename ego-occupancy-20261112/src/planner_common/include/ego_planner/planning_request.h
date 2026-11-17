#pragma once

#include <ego_planner/planning_types.h>

#include <optional>
#include <vector>

namespace ego_planner {

struct BoundaryState {
  Vec3 position = Vec3::Zero();
  Vec3 velocity = Vec3::Zero();
  Vec3 acceleration = Vec3::Zero();
};

/// 人工开口坐标：normal指向穿越方向，u/v为窗口正交单位轴。
struct Gate {
  Vec3 center = Vec3::Zero();
  Vec3 normal = Vec3::UnitX();
  Vec3 u = Vec3::UnitY();
  Vec3 v = Vec3::UnitZ();
  double half_width = 0.5;
  double half_height = 0.5;
};

struct PlanningRequest {
  std::uint64_t request_id = 1;
  std::uint64_t goal_version = 1;
  BoundaryState start;
  BoundaryState goal;
  std::vector<Vec3> reference_points;
  std::optional<Gate> gate;
};

}  // namespace ego_planner
