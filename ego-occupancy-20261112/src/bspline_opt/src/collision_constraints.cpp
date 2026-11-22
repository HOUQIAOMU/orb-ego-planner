#include <ego_planner/collision_constraints.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner {
const char* toString(ConstraintStatus s) {
  switch (s) {
    case ConstraintStatus::NoCollision:
      return "NoCollision";
    case ConstraintStatus::ConstraintsReady:
      return "ConstraintsReady";
    case ConstraintStatus::SearchFailed:
      return "SearchFailed";
    case ConstraintStatus::DegenerateConstraint:
      return "DegenerateConstraint";
    case ConstraintStatus::ResourceLimit:
      return "ResourceLimit";
    case ConstraintStatus::Timeout:
      return "Timeout";
    case ConstraintStatus::Cancelled:
      return "Cancelled";
  }
  return "DegenerateConstraint";
}

ConstraintResult CollisionConstraintBuilder::build(const PlanningMapView& view,
                                                   const Bspline& spline, const PlanningConfig& c,
                                                   WorkBudget budget,
                                                   const std::vector<int>& forced) const {
  ConstraintResult result;
  auto finish = [&](ConstraintStatus status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    if (status != ConstraintStatus::ConstraintsReady) result.constraints.clear();
    return result;
  };
  auto interrupted = [&]() {
    return budget.status() == WorkStatus::Cancelled ? ConstraintStatus::Cancelled
                                                    : ConstraintStatus::Timeout;
  };
  if (budget.status() != WorkStatus::Ready)
    return finish(interrupted(), "constraint construction interrupted");
  const double dt = uniformInterval(spline), duration = spline.duration();
  const auto& q = spline.controlPoints();
  const double resolution = view.snapshot().resolution();
  // 此处采样只用于发现碰撞和构造代理；最终安全性由独立Bezier验收器决定。
  const double samples_needed =
      std::ceil(spline.normBounds().first * duration / (.35 * resolution));
  if (!std::isfinite(samples_needed) ||
      samples_needed > static_cast<double>(c.max_validation_visits))
    return finish(ConstraintStatus::ResourceLimit, "collision discovery exceeds sample bound");
  const int count = std::max(static_cast<int>(samples_needed), static_cast<int>(q.cols()) * 8);
  if (static_cast<std::size_t>(count + 1) > c.max_validation_visits)
    return finish(ConstraintStatus::ResourceLimit, "collision discovery exceeds sample bound");
  std::vector<Vec3> positions;
  std::vector<bool> blocked;
  positions.reserve(count + 1);
  blocked.reserve(count + 1);
  for (int i = 0; i <= count; ++i) {
    if (budget.status() != WorkStatus::Ready)
      return finish(interrupted(), "collision sampling interrupted");
    const double time = duration * i / count;
    positions.push_back(spline.evaluate(time));
    const int span = std::min(static_cast<int>(time / dt), static_cast<int>(q.cols()) - 4);
    const bool force = std::find(forced.begin(), forced.end(), span) != forced.end();
    blocked.push_back(!view.pointFree(positions.back()) || (force && i > 0 && i < count));
  }
  if (blocked.front() || blocked.back())
    return finish(ConstraintStatus::SearchFailed,
                  "start or goal is blocked; endpoints are not moved");
  for (int i = 1; i < count; ++i) {
    if (!blocked[i]) continue;
    const int first = i;
    while (i < count && blocked[i]) ++i;
    const int last = i - 1;
    int entry = first - 1, exit = i;
    // 给局部A*与碰撞曲线留一个采样段的过渡空间。
    if (entry > 0 && !blocked[entry - 1]) --entry;
    if (exit < count && !blocked[exit + 1]) ++exit;
    ++result.astar_calls;
    SearchOptions options;
    options.max_nodes = c.max_roi_voxels;
    options.max_expansions = c.max_roi_voxels;
    options.max_queue_entries = c.max_roi_voxels * 4;
    auto path = Astar{}.search(view, positions[entry], positions[exit], options, budget);
    if (path.status != SearchStatus::Success) {
      if (budget.status() != WorkStatus::Ready)
        return finish(interrupted(), "segment A* interrupted");
      return finish(path.status == SearchStatus::ResourceLimit ? ConstraintStatus::ResourceLimit
                                                               : ConstraintStatus::SearchFailed,
                    std::string("segment A*: ") + toString(path.status));
    }
    result.segments.push_back({duration * entry / count, duration * exit / count, path.path});
    const int begin_control = std::max(3, static_cast<int>((duration * first / count) / dt));
    const int end_control = std::min(static_cast<int>(q.cols()) - 4,
                                     static_cast<int>((duration * last / count) / dt) + 3);
    const auto before = result.constraints.size();
    // 薄墙可能在两个控制点之间，所有控制点都Free。使用曲线碰撞见证构造
    // 一条真实绕障边界，并传播给该段支撑控制点（原EGO短段处理的推广）。
    int witness = -1;
    for (int k = first; k <= last; ++k)
      if (!view.pointFree(positions[k])) {
        if (witness < 0 ||
            std::abs(k - (first + last) / 2) < std::abs(witness - (first + last) / 2))
          witness = k;
      }
    if (witness >= 0) {
      const Vec3 origin = positions[witness];
      const Vec3 tangent = spline.derivative().evaluate(duration * witness / count);
      Vec3 target = Vec3::Zero();
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t k = 1; k < path.path.size(); ++k) {
        const double a = (path.path[k - 1] - origin).dot(tangent);
        const double b = (path.path[k] - origin).dot(tangent);
        if (((a <= 0 && b >= 0) || (a >= 0 && b <= 0)) && std::abs(a - b) > 1e-12) {
          const Vec3 point = path.path[k - 1] + a / (a - b) * (path.path[k] - path.path[k - 1]);
          if ((point - origin).squaredNorm() < best) {
            best = (point - origin).squaredNorm();
            target = point;
          }
        }
      }
      if (std::isfinite(best) && best > 1e-16 && view.pointFree(target)) {
        const double length = (target - origin).norm();
        const Vec3 direction = (target - origin) / length;
        Vec3 base = target;
        const double needed = std::ceil(length / (.25 * resolution));
        if (!std::isfinite(needed) || needed > c.max_validation_visits)
          return finish(ConstraintStatus::ResourceLimit, "witness ray exceeds visit bound");
        const int steps = std::max(1, static_cast<int>(needed));
        for (int k = 1; k <= steps; ++k) {
          if (budget.status() != WorkStatus::Ready)
            return finish(interrupted(), "witness ray interrupted");
          const Vec3 point = target - length * k / steps * direction;
          if (!view.pointFree(point)) break;
          base = point;
        }
        for (int j = begin_control; j <= end_control; ++j)
          result.constraints.push_back({j, base, direction});
      }
    }
    for (int j = begin_control; j <= end_control; ++j) {
      if (budget.status() != WorkStatus::Ready)
        return finish(interrupted(), "proxy construction interrupted");
      const Vec3 point = q.col(j), tangent = q.col(j + 1) - q.col(j - 1);
      Vec3 target = Vec3::Zero();
      double best = std::numeric_limits<double>::infinity();
      // 沿局部轨迹切平面寻找A*路径交点，保留原EGO的base/direction思路。
      if (tangent.norm() > 1e-9)
        for (std::size_t k = 1; k < path.path.size(); ++k) {
          const double a = (path.path[k - 1] - point).dot(tangent);
          const double b = (path.path[k] - point).dot(tangent);
          if ((a <= 0 && b >= 0) || (a >= 0 && b <= 0)) {
            const double denominator = a - b;
            if (std::abs(denominator) < 1e-12) continue;
            const Vec3 p = path.path[k - 1] + (a / denominator) * (path.path[k] - path.path[k - 1]);
            if ((p - point).squaredNorm() < best) {
              best = (p - point).squaredNorm();
              target = p;
            }
          }
        }
      if (!std::isfinite(best)) {
        for (const auto& p : path.path)
          if ((p - point).squaredNorm() < best) {
            best = (p - point).squaredNorm();
            target = p;
          }
      }
      const double distance = (target - point).norm();
      if (!std::isfinite(distance) || distance < 1e-8) continue;
      const Vec3 direction = (target - point) / distance;
      if (!view.pointFree(target)) continue;
      Vec3 base = target;
      const double steps_needed = std::ceil(distance / (.25 * resolution));
      if (!std::isfinite(steps_needed) || steps_needed > c.max_validation_visits)
        return finish(ConstraintStatus::ResourceLimit, "proxy ray exceeds visit bound");
      const int steps = std::max(1, static_cast<int>(steps_needed));
      for (int k = 1; k <= steps; ++k) {
        if (budget.status() != WorkStatus::Ready)
          return finish(interrupted(), "proxy ray interrupted");
        const Vec3 p = target - (distance * k / steps) * direction;
        if (!view.pointFree(p)) break;
        base = p;
      }
      result.constraints.push_back({j, base, direction});
    }
    if (result.constraints.size() == before)
      return finish(ConstraintStatus::DegenerateConstraint,
                    "collision segment produced no movable proxy");
  }
  if (result.segments.empty())
    return finish(ConstraintStatus::NoCollision, "no collision discovered");
  return finish(ConstraintStatus::ConstraintsReady, "collision constraints ready");
}
}  // namespace ego_planner
