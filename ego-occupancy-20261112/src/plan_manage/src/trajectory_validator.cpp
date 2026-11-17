#include <ego_planner/trajectory_validator.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner {
const char* toString(ValidationStatus s) {
  switch (s) {
    case ValidationStatus::NotRun:
      return "NotRun";
    case ValidationStatus::Valid:
      return "Valid";
    case ValidationStatus::InvalidInput:
      return "InvalidInput";
    case ValidationStatus::BoundaryMismatch:
      return "BoundaryMismatch";
    case ValidationStatus::Collision:
      return "Collision";
    case ValidationStatus::Unverified:
      return "Unverified";
    case ValidationStatus::DynamicsExceeded:
      return "DynamicsExceeded";
    case ValidationStatus::GateNotSatisfied:
      return "GateNotSatisfied";
    case ValidationStatus::Timeout:
      return "Timeout";
    case ValidationStatus::Cancelled:
      return "Cancelled";
  }
  return "InvalidInput";
}
namespace {
std::pair<double, double> SpanTimes(const Bspline& spline, int span) {
  uniformInterval(spline);
  if (span < 0 || span >= spline.controlPoints().cols() - 3)
    throw std::invalid_argument("invalid Bezier span index");
  const auto& knots = spline.knots();
  return {knots[span + 3] - knots[3], knots[span + 4] - knots[3]};
}
}  // namespace
BezierSpan bezierSpan(const Bspline& spline, int span) {
  const auto times = SpanTimes(spline, span);
  const auto velocity = spline.derivative();
  const double h = times.second - times.first;
  const Vec3 p = spline.evaluate(times.first), q = spline.evaluate(times.second);
  // 使用同一条原样条的真实节点区间、端点与导数，证明对象与evaluate保持一致。
  return {p, p + h / 3 * velocity.evaluate(times.first),
          q - h / 3 * velocity.evaluate(times.second), q};
}
namespace {
struct CurveChecker {
  const PlanningMapView& view;
  const PlanningConfig& config;
  WorkBudget budget;
  ValidationReport& report;

  bool work() {
    if (budget.status() != WorkStatus::Ready) {
      report.status = budget.status() == WorkStatus::Cancelled ? ValidationStatus::Cancelled
                                                               : ValidationStatus::Timeout;
      return false;
    }
    if (++report.visits > config.max_validation_visits) {
      report.status = ValidationStatus::Unverified;
      return false;
    }
    return true;
  }
  bool boxFree(const BezierSpan& points) {
    Vec3 low = points[0], high = points[0];
    for (const auto& p : points) {
      low = low.cwiseMin(p);
      high = high.cwiseMax(p);
    }
    const Vec3 a = (low - view.snapshot().origin()) / view.snapshot().resolution();
    const Vec3 b = (high - view.snapshot().origin()) / view.snapshot().resolution();
    Index minimum, maximum;
    for (int axis = 0; axis < 3; ++axis) {
      // AABB按闭集查询，体素边界同时覆盖两侧；绝不先将巨大double转成int。
      // Bezier加权和及坐标转换可累积数个ULP，单次nextafter不足以覆盖。
      const double roundoff =
          64 * std::numeric_limits<double>::epsilon() *
          std::max({1.0, std::abs(a[axis]), std::abs(b[axis]),
                    std::abs(view.snapshot().origin()[axis] / view.snapshot().resolution())});
      const double l = std::floor(a[axis] - roundoff);
      const double h = std::floor(b[axis] + roundoff);
      if (!std::isfinite(l) || !std::isfinite(h) || l < view.roi().minimum[axis] ||
          h >= view.roi().maximum_exclusive[axis])
        return false;
      minimum[axis] = static_cast<int>(l);
      maximum[axis] = static_cast<int>(h);
    }
    for (int x = minimum.x(); x <= maximum.x(); ++x)
      for (int y = minimum.y(); y <= maximum.y(); ++y)
        for (int z = minimum.z(); z <= maximum.z(); ++z) {
          if (!work()) return false;
          if (view.blocked(Index(x, y, z))) return false;
        }
    return true;
  }
  bool check(const BezierSpan& p, int span, double begin, double end, int depth) {
    if (!work()) return false;
    const Vec3 a = (p[0] + p[1]) / 2, b = (p[1] + p[2]) / 2, c = (p[2] + p[3]) / 2;
    const Vec3 d = (a + b) / 2, e = (b + c) / 2, m = (d + e) / 2;
    if (!view.pointFree(p[0]) || !view.pointFree(m) || !view.pointFree(p[3])) {
      report.status = ValidationStatus::Collision;
      report.collision_span = span;
      report.collision_begin = begin;
      report.collision_end = end;
      return false;
    }
    if (boxFree(p)) return true;
    if (report.status == ValidationStatus::Unverified ||
        report.status == ValidationStatus::Timeout || report.status == ValidationStatus::Cancelled)
      return false;
    if (depth >= config.max_subdivision_depth) {
      report.status = ValidationStatus::Unverified;
      return false;
    }
    const double middle = (begin + end) / 2;
    const bool left = check({p[0], a, d, m}, span, begin, middle, depth + 1);
    if (!left && report.status != ValidationStatus::Unverified) return false;
    // 左侧边界舍入可能只能判为Unverified；仍看右侧见证，避免遮蔽已存在的碰撞。
    // 未证明的左侧始终保持拒绝，右侧成功不能把整段升级为Valid。
    const bool right = check({m, e, c, p[3]}, span, middle, end, depth + 1);
    return left && right;
  }
};

/// 三次多项式先按导数根分成单调区间，再夹逼全部平面交点。
/// 近共面和切线均拒绝，避免把数值不确定的接触当作一次可靠穿门。
bool CheckGate(const Bspline& spline, const Gate& gate, const PlanningConfig& config,
               WorkBudget budget, ValidationReport& report) {
  uniformInterval(spline);
  const double tol = config.gate_tolerance;
  std::vector<double> crossings;
  const int spans = static_cast<int>(spline.controlPoints().cols()) - 3;
  for (int s = 0; s < spans; ++s) {
    if (budget.status() != WorkStatus::Ready) return false;
    const auto times = SpanTimes(spline, s);
    const double dt = times.second - times.first;
    const auto p = bezierSpan(spline, s);
    double d[4];
    for (int i = 0; i < 4; ++i) d[i] = (p[i] - gate.center).dot(gate.normal);
    if (*std::min_element(d, d + 4) > tol || *std::max_element(d, d + 4) < -tol) continue;
    const double a = -d[0] + 3 * d[1] - 3 * d[2] + d[3];
    const double b = 3 * d[0] - 6 * d[1] + 3 * d[2], c = -3 * d[0] + 3 * d[1], offset = d[0];
    auto f = [&](double u) { return ((a * u + b) * u + c) * u + offset; };
    std::vector<double> limits{0, 1};
    if (std::abs(a) > 1e-12) {
      const double discriminant = 4 * b * b - 12 * a * c;
      if (discriminant >= 0) {
        const double root = std::sqrt(discriminant);
        for (double u : {(-2 * b - root) / (6 * a), (-2 * b + root) / (6 * a)})
          if (u > 0 && u < 1) limits.push_back(u);
      }
    } else if (std::abs(b) > 1e-12) {
      const double u = -c / (2 * b);
      if (u > 0 && u < 1) limits.push_back(u);
    }
    std::sort(limits.begin(), limits.end());
    limits.erase(std::unique(limits.begin(), limits.end(),
                             [](double x, double y) { return std::abs(x - y) < 1e-12; }),
                 limits.end());
    auto add = [&](double u) {
      const double derivative = (3 * a * u + 2 * b) * u + c;
      if (derivative <= tol) return false;
      const double time = times.first + u * dt;
      if (crossings.empty() || std::abs(time - crossings.back()) > 1e-7 * std::max(1.0, dt))
        crossings.push_back(time);
      return true;
    };
    for (double u : limits)
      if (std::abs(f(u)) <= tol && !add(u)) return false;
    for (std::size_t i = 1; i < limits.size(); ++i) {
      double low = limits[i - 1], high = limits[i], fl = f(low), fh = f(high);
      if (std::abs(fl) <= tol || std::abs(fh) <= tol || (fl < 0) == (fh < 0)) continue;
      for (int iteration = 0; iteration < 50; ++iteration) {
        if (budget.status() != WorkStatus::Ready) return false;
        const double mid = (low + high) / 2, fm = f(mid);
        if ((fm < 0) == (fl < 0)) {
          low = mid;
          fl = fm;
        } else
          high = mid;
      }
      if (!add((low + high) / 2)) return false;
    }
  }
  std::sort(crossings.begin(), crossings.end());
  crossings.erase(std::unique(crossings.begin(), crossings.end(),
                              [](double a, double b) { return std::abs(a - b) < 1e-7; }),
                  crossings.end());
  report.gate_crossings = static_cast<int>(crossings.size());
  if (crossings.size() != 1 || (spline.evaluate(0) - gate.center).dot(gate.normal) >= -tol ||
      (spline.evaluate(spline.duration()) - gate.center).dot(gate.normal) <= tol)
    return false;
  const Vec3 local = spline.evaluate(crossings.front()) - gate.center;
  const double width = gate.half_width - gate.u.cwiseAbs().dot(config.half_extent);
  const double height = gate.half_height - gate.v.cwiseAbs().dot(config.half_extent);
  return std::abs(local.dot(gate.u)) < width - tol && std::abs(local.dot(gate.v)) < height - tol;
}
}  // namespace

ValidationReport TrajectoryValidator::validate(const PlanningMapView& view, const Bspline& spline,
                                               const PlanningRequest& r, const PlanningConfig& c,
                                               WorkBudget budget) const {
  ValidationReport report;
  auto finish = [&](ValidationStatus status, const std::string& reason) {
    report.status = status;
    report.reason = reason;
    return report;
  };
  auto interrupted = [&]() {
    return budget.status() == WorkStatus::Cancelled ? ValidationStatus::Cancelled
                                                    : ValidationStatus::Timeout;
  };
  if (budget.status() != WorkStatus::Ready) return finish(interrupted(), "validation interrupted");
  try {
    validateConfig(c);
    if ((view.halfExtent().array() != c.half_extent.array()).any())
      return finish(ValidationStatus::InvalidInput, "view/config body geometry mismatch");
    auto geometry_request = r;
    geometry_request.gate.reset();
    validateRequest(geometry_request, c);
    if (r.gate) {
      try {
        validateRequest(r, c);
      } catch (const std::invalid_argument& e) {
        return finish(ValidationStatus::GateNotSatisfied, e.what());
      }
    }
    uniformInterval(spline);
    const auto velocity = spline.derivative(), acceleration = velocity.derivative();
    for (int side = 0; side < 2; ++side) {
      const double time = side ? spline.duration() : 0;
      const auto& state = side ? r.goal : r.start;
      report.boundary_error =
          std::max({report.boundary_error, (spline.evaluate(time) - state.position).norm(),
                    (velocity.evaluate(time) - state.velocity).norm(),
                    (acceleration.evaluate(time) - state.acceleration).norm()});
    }
    if (!std::isfinite(report.boundary_error) || report.boundary_error > c.boundary_tolerance)
      return finish(ValidationStatus::BoundaryMismatch, "endpoint p/v/a mismatch");
    const auto bounds = spline.normBounds();
    report.speed_bound = bounds.first;
    report.acceleration_bound = bounds.second;
    if (bounds.first > c.max_velocity * (1 + 1e-9) ||
        bounds.second > c.max_acceleration * (1 + 1e-9))
      return finish(ValidationStatus::DynamicsExceeded,
                    "3D derivative convex-hull bound exceeds limit");
    report.status = ValidationStatus::Valid;
    CurveChecker checker{view, c, budget, report};
    for (int i = 0; i < spline.controlPoints().cols() - 3; ++i) {
      const auto times = SpanTimes(spline, i);
      if (!checker.check(bezierSpan(spline, i), i, times.first, times.second, 0))
        return finish(report.status, "whole-curve conservative collision validation rejected");
    }
    if (r.gate && !CheckGate(spline, *r.gate, c, budget, report)) {
      if (budget.status() != WorkStatus::Ready)
        return finish(interrupted(), "gate validation interrupted");
      return finish(ValidationStatus::GateNotSatisfied,
                    "expected one forward crossing inside body-shrunk gate");
    }
    if (budget.status() != WorkStatus::Ready)
      return finish(interrupted(), "validation interrupted");
    return finish(ValidationStatus::Valid,
                  "whole curve, endpoint states, dynamics and gate verified");
  } catch (const std::exception& e) {
    return finish(ValidationStatus::InvalidInput, e.what());
  }
}
}  // namespace ego_planner
