#include <ego_planner/planning_map_view.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ego_planner {
namespace {
void CheckBudget(const WorkBudget& budget) {
  if (budget.status() == WorkStatus::Cancelled) throw std::runtime_error("map view cancelled");
  if (budget.status() == WorkStatus::Timeout) throw std::runtime_error("map view timeout");
}
}  // namespace
PlanningMapView::PlanningMapView(std::shared_ptr<const ego_occupancy::MapSnapshot> snapshot,
                                 Roi roi, const Vec3& half_extent, WorkBudget budget,
                                 std::size_t maximum_voxels)
    : snapshot_(std::move(snapshot)), roi_(std::move(roi)), half_extent_(half_extent) {
  CheckBudget(budget);
  if (!snapshot_ || !half_extent.allFinite() || (half_extent.array() < 0).any())
    throw std::invalid_argument("invalid snapshot or body half extent");
  if ((roi_.minimum.array() < 0).any() ||
      (roi_.maximum_exclusive.array() > snapshot_->dimensions().array()).any() ||
      (roi_.maximum_exclusive.array() <= roi_.minimum.array()).any())
    throw std::invalid_argument("ROI outside snapshot");
  point_geometry_ = (half_extent.array() == 0).all();
  size_ = roi_.maximum_exclusive - roi_.minimum;
  std::size_t count = 1;
  for (int axis = 0; axis < 3; ++axis) {
    if (static_cast<std::size_t>(size_[axis]) > maximum_voxels / count)
      throw std::length_error("planning ROI exceeds voxel budget");
    count *= static_cast<std::size_t>(size_[axis]);
  }
  blocked_.resize(count);
  std::size_t visited = 0;
  for (int z = roi_.minimum.z(); z < roi_.maximum_exclusive.z(); ++z)
    for (int y = roi_.minimum.y(); y < roi_.maximum_exclusive.y(); ++y)
      for (int x = roi_.minimum.x(); x < roi_.maximum_exclusive.x(); ++x) {
        Index index(x, y, z);
        blocked_[offset(index)] = snapshot_->query(index).state != ego_occupancy::VoxelState::Free;
        if ((++visited & 4095U) == 0) CheckBudget(budget);
      }
  // 三次一维滑动窗口完成盒形膨胀，O(ROI体素数)，不按机体半径重复遍历。
  // ROI之外视为阻挡，避免外廓越出已验证范围；此处不读取原inflated层。
  for (int axis = 0; axis < 3; ++axis) {
    // 候选体素和非零外廓均按闭集处理。整数倍半尺寸必须多覆盖接触面那一格。
    const double ratio = half_extent[axis] / snapshot_->resolution();
    const double cells =
        point_geometry_
            ? 0.0
            : std::floor(std::nextafter(ratio, std::numeric_limits<double>::infinity())) + 1.0;
    if (!std::isfinite(cells) || cells >= size_[axis]) {
      std::fill(blocked_.begin(), blocked_.end(), 1);
      break;
    }
    const int radius = static_cast<int>(cells);
    if (radius == 0) continue;
    const auto input = blocked_;
    for (int z = 0; z < size_.z(); ++z)
      for (int y = 0; y < size_.y(); ++y)
        for (int x = 0; x < size_.x(); ++x) {
          Index start(x, y, z);
          if (start[axis] != 0) continue;
          auto value = [&](int position) {
            if (position < 0 || position >= size_[axis]) return 1;
            Index local = start;
            local[axis] = position;
            return static_cast<int>(input[offset(local + roi_.minimum)]);
          };
          int sum = 0;
          for (int j = -radius; j <= radius; ++j) sum += value(j);
          for (int i = 0; i < size_[axis]; ++i) {
            Index local = start;
            local[axis] = i;
            blocked_[offset(local + roi_.minimum)] = sum > 0;
            sum -= value(i - radius);
            sum += value(i + radius + 1);
            if ((++visited & 4095U) == 0) CheckBudget(budget);
          }
        }
  }
  CheckBudget(budget);
}
std::size_t PlanningMapView::offset(const Index& index) const {
  const Index p = index - roi_.minimum;
  return static_cast<std::size_t>(p.x()) +
         static_cast<std::size_t>(size_.x()) *
             (static_cast<std::size_t>(p.y()) + static_cast<std::size_t>(size_.y()) * p.z());
}
bool PlanningMapView::blocked(const Index& index) const {
  if ((index.array() < roi_.minimum.array()).any() ||
      (index.array() >= roi_.maximum_exclusive.array()).any())
    return true;
  return blocked_[offset(index)] != 0;
}
bool PlanningMapView::pointFree(const Vec3& point) const {
  const auto center = snapshot_->worldToIndex(point);
  if (!center || blocked(*center)) return false;
  if (!point_geometry_) return true;  // 非零外廓由闭合体素掩码保守覆盖。
  const Vec3 grid = (point - snapshot_->origin()) / snapshot_->resolution();
  Index minimum, maximum;
  for (int axis = 0; axis < 3; ++axis) {
    const double low =
        std::floor(std::nextafter(grid[axis], -std::numeric_limits<double>::infinity()));
    const double high =
        std::floor(std::nextafter(grid[axis], std::numeric_limits<double>::infinity()));
    if (low < roi_.minimum[axis] || high >= roi_.maximum_exclusive[axis]) return false;
    minimum[axis] = static_cast<int>(low);
    maximum[axis] = static_cast<int>(high);
  }
  for (int x = minimum.x(); x <= maximum.x(); ++x)
    for (int y = minimum.y(); y <= maximum.y(); ++y)
      for (int z = minimum.z(); z <= maximum.z(); ++z)
        if (snapshot_->query(Index(x, y, z)).state != ego_occupancy::VoxelState::Free) return false;
  return true;
}
bool PlanningMapView::edgeFree(const Index& from, const Index& to) const {
  // 仅接受同格或26邻接；所有轴组合都需Free，保守禁止对角穿角。
  const auto delta = (to.cast<long long>() - from.cast<long long>()).eval();
  if ((delta.array().abs() > 1).any() || blocked(from) || blocked(to)) return false;
  for (int bits = 0; bits < 8; ++bits) {
    Index index = from;
    for (int axis = 0; axis < 3; ++axis)
      if (bits & (1 << axis)) index[axis] = to[axis];
    if (blocked(index)) return false;
  }
  return true;
}
}  // namespace ego_planner
