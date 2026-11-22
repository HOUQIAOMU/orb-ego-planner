#pragma once
#include <ego_occupancy/map.h>

#include <string>

namespace ego_occupancy {
/// 显示边界只接收自有数值数组，不持规划器或地图租约。
struct PlanningLayer {
  bool has_task = false;
  bool ready = false;
  std::uint64_t version = 0;
  std::string status;
  std::string reason;
  Vec3 start = Vec3::Zero();
  Vec3 goal = Vec3::Zero();
  std::vector<Vec3> initial;
  std::vector<std::vector<Vec3>> astar;
  std::vector<Vec3> trajectory;
};
}  // namespace ego_occupancy
