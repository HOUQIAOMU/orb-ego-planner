#include <ego_planner/planning_config.h>

#include <cmath>
#include <stdexcept>

namespace ego_planner {
namespace {
bool Positive(double value) { return std::isfinite(value) && value > 0.0; }
bool Finite(const BoundaryState& state) {
  return state.position.allFinite() && state.velocity.allFinite() && state.acceleration.allFinite();
}
}  // namespace

void validateConfig(const PlanningConfig& c) {
  if (!c.half_extent.allFinite() || (c.half_extent.array() < 0).any() ||
      !Positive(c.nominal_speed) || !Positive(c.max_velocity) || !Positive(c.max_acceleration) ||
      c.nominal_speed > c.max_velocity || !Positive(c.control_point_spacing) ||
      !Positive(c.roi_margin) || !Positive(c.soft_clearance) || !Positive(c.smoothness_weight) ||
      !Positive(c.collision_weight) || !Positive(c.feasibility_weight) ||
      !Positive(c.fitness_weight) || !Positive(c.boundary_tolerance) ||
      !Positive(c.gate_tolerance) || !Positive(c.max_duration) || c.max_duration > 600.0 ||
      c.max_control_points < 8 || c.max_control_points > 256 || c.max_roi_voxels == 0 ||
      c.max_roi_voxels > 4'000'000 || c.max_validation_visits == 0 ||
      c.max_validation_visits > 10'000'000 || c.max_subdivision_depth < 1 ||
      c.max_subdivision_depth > 30 || c.max_iterations < 1 || c.max_iterations > 1000 ||
      c.max_restarts < 0 || c.max_restarts > 10 || c.max_time_adjustments < 0 ||
      c.max_time_adjustments > 8)
    throw std::invalid_argument("invalid planning configuration or resource bound");
}

void validateRequest(const PlanningRequest& r, const PlanningConfig& c) {
  validateConfig(c);
  if (!r.request_id || !r.goal_version || !Finite(r.start) || !Finite(r.goal) ||
      r.reference_points.size() > 7)
    throw std::invalid_argument("invalid request identity, boundary or reference count");
  Vec3 previous = r.start.position;
  double length = 0.0;
  for (const auto& point : r.reference_points) {
    if (!point.allFinite() || (point - previous).norm() < 1e-5)
      throw std::invalid_argument("nonfinite or repeated reference point");
    length += (point - previous).norm();
    previous = point;
  }
  if ((r.goal.position - previous).norm() < 1e-5)
    throw std::invalid_argument("zero length reference segment");
  length += (r.goal.position - previous).norm();
  if (!std::isfinite(length) || length > 100.0)
    throw std::invalid_argument("reference length exceeds experiment limit");
  if (r.start.velocity.norm() > c.max_velocity || r.goal.velocity.norm() > c.max_velocity ||
      r.start.acceleration.norm() > c.max_acceleration ||
      r.goal.acceleration.norm() > c.max_acceleration)
    throw std::invalid_argument("requested boundary exceeds experiment dynamics");
  if (r.gate) {
    const auto& g = *r.gate;
    if (!g.center.allFinite() || !g.normal.allFinite() || !g.u.allFinite() || !g.v.allFinite() ||
        std::abs(g.normal.norm() - 1.0) > 1e-6 || std::abs(g.u.norm() - 1.0) > 1e-6 ||
        std::abs(g.v.norm() - 1.0) > 1e-6 || std::abs(g.u.dot(g.v)) > 1e-6 ||
        std::abs(g.u.dot(g.normal)) > 1e-6 || std::abs(g.v.dot(g.normal)) > 1e-6 ||
        !Positive(g.half_width) || !Positive(g.half_height) ||
        g.half_width <= g.u.cwiseAbs().dot(c.half_extent) + c.gate_tolerance ||
        g.half_height <= g.v.cwiseAbs().dot(c.half_extent) + c.gate_tolerance ||
        (r.start.position - g.center).dot(g.normal) >= -c.gate_tolerance ||
        (r.goal.position - g.center).dot(g.normal) <= c.gate_tolerance)
      throw std::invalid_argument("invalid gate frame, clearance or traversal direction");
  }
}

}  // namespace ego_planner
