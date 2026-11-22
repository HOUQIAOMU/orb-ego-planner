#pragma once
#include <ego_planner/planning_types.h>

#include <memory>
#include <vector>

namespace ego_planner {
/// 单轮只读查询：只从raw三态生成配置空间，不重复叠加原地图膨胀。
class PlanningMapView {
 public:
  PlanningMapView(std::shared_ptr<const ego_occupancy::MapSnapshot> snapshot, Roi roi,
                  const Vec3& half_extent, WorkBudget budget = {},
                  std::size_t maximum_voxels = 4'000'000);
  bool blocked(const Index& index) const;
  /// 非格中心端点也检查闭合接触；零外廓点位于格边界时不能漏掉相邻非Free格。
  bool pointFree(const Vec3& point) const;
  bool edgeFree(const Index& from, const Index& to) const;
  const ego_occupancy::MapSnapshot& snapshot() const { return *snapshot_; }
  const Roi& roi() const { return roi_; }
  const Vec3& halfExtent() const { return half_extent_; }
  std::size_t voxelCount() const { return blocked_.size(); }

 private:
  std::size_t offset(const Index& index) const;
  std::shared_ptr<const ego_occupancy::MapSnapshot> snapshot_;
  Roi roi_;
  Vec3 half_extent_;
  bool point_geometry_ = false;
  Index size_;
  std::vector<std::uint8_t> blocked_;
};
}  // namespace ego_planner
