#include <ego_planner/planner_core.h>

#include <algorithm>
#include <cmath>
#include <new>

namespace ego_planner {
const char* toString(CandidateSource source) {
  switch (source) {
    case CandidateSource::None:
      return "None";
    case CandidateSource::Optimized:
      return "Optimized";
    case CandidateSource::SeedFallback:
      return "SeedFallback";
  }
  return "None";
}
const char* toString(PlanStatus s) {
  switch (s) {
    case PlanStatus::Success:
      return "Success";
    case PlanStatus::InvalidInput:
      return "InvalidInput";
    case PlanStatus::StartBlocked:
      return "StartBlocked";
    case PlanStatus::GoalBlocked:
      return "GoalBlocked";
    case PlanStatus::ConstraintFailure:
      return "ConstraintFailure";
    case PlanStatus::OptimizerFailure:
      return "OptimizerFailure";
    case PlanStatus::ValidationFailure:
      return "ValidationFailure";
    case PlanStatus::NoPath:
      return "NoPath";
    case PlanStatus::ResourceLimit:
      return "ResourceLimit";
    case PlanStatus::Timeout:
      return "Timeout";
    case PlanStatus::Cancelled:
      return "Cancelled";
  }
  return "InvalidInput";
}
namespace {
double Elapsed(Clock::time_point from) {
  return std::chrono::duration<double, std::milli>(Clock::now() - from).count();
}
Roi MakeRoi(const ego_occupancy::MapSnapshot& map, const Bspline& seed,
            const PlanningConfig& config, int attempt) {
  Vec3 low = seed.controlPoints().rowwise().minCoeff(),
       high = seed.controlPoints().rowwise().maxCoeff();
  const Vec3 margin = Vec3::Constant(config.roi_margin * (attempt + 1) + 2 * map.resolution()) +
                      2 * config.half_extent;
  low -= margin;
  high += margin;
  const Vec3 minimum = (low - map.origin()) / map.resolution(),
             maximum = (high - map.origin()) / map.resolution();
  Roi roi;
  for (int axis = 0; axis < 3; ++axis) {
    const double l = std::floor(minimum[axis]), h = std::ceil(maximum[axis]) + 1;
    if (!std::isfinite(l) || !std::isfinite(h))
      throw std::invalid_argument("nonfinite planning ROI");
    roi.minimum[axis] =
        static_cast<int>(std::clamp(l, 0.0, static_cast<double>(map.dimensions()[axis])));
    roi.maximum_exclusive[axis] =
        static_cast<int>(std::clamp(h, 0.0, static_cast<double>(map.dimensions()[axis])));
  }
  return roi;
}
}  // namespace

PlanResult PlannerCore::plan(std::shared_ptr<const ego_occupancy::MapSnapshot> snapshot,
                             const PlanningRequest& request, const PlanningConfig& config,
                             WorkBudget budget) const {
  const auto began = Clock::now();
  PlanResult result;
  auto finish = [&](PlanStatus status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    result.timings.total_ms = Elapsed(began);
    if (status != PlanStatus::Success) result.trajectory.reset();
    return result;
  };
  auto interrupted = [&]() {
    return budget.status() == WorkStatus::Cancelled ? PlanStatus::Cancelled : PlanStatus::Timeout;
  };
  if (budget.status() != WorkStatus::Ready)
    return finish(interrupted(), "request budget unavailable");
  if (!snapshot) return finish(PlanStatus::InvalidInput, "no map snapshot");
  result.planned_on = snapshot->metadata();
  try {
    validateRequest(request, config);
    auto time = Clock::now();
    const auto seed = SeedBuilder{}.build(request, config);
    result.timings.seed_ms = Elapsed(time);
    result.initial_spline = seed.spline;
    result.reference = seed.reference;
    const int attempts = config.allow_roi_expansion ? 2 : 1;
    for (int attempt = 0; attempt < attempts; ++attempt) {
      if (budget.status() != WorkStatus::Ready)
        return finish(interrupted(), "planning interrupted");
      result.roi = MakeRoi(*snapshot, seed.spline, config, attempt);
      ++result.roi_attempts;
      time = Clock::now();
      PlanningMapView view(snapshot, result.roi, config.half_extent, budget, config.max_roi_voxels);
      result.timings.view_ms += Elapsed(time);
      if (!view.pointFree(request.start.position))
        return finish(PlanStatus::StartBlocked, "start is not free in configuration space");
      if (!view.pointFree(request.goal.position))
        return finish(PlanStatus::GoalBlocked, "goal is not free in configuration space");
      // 保留已完整证明可行的初值，防止平滑优化丢弃人工门的合法路线。
      time = Clock::now();
      const auto seed_validation =
          TrajectoryValidator{}.validate(view, seed.spline, request, config, budget);
      result.timings.validation_ms += Elapsed(time);
      if (budget.status() != WorkStatus::Ready)
        return finish(interrupted(), "seed validation interrupted");
      const bool feasible_seed = seed_validation.status == ValidationStatus::Valid;
      auto reject = [&](PlanStatus status, const std::string& reason) {
        if (feasible_seed && budget.status() == WorkStatus::Ready) {
          const auto validation_started = Clock::now();
          const auto fallback =
              TrajectoryValidator{}.validate(view, seed.spline, request, config, budget);
          result.timings.validation_ms += Elapsed(validation_started);
          if (budget.status() != WorkStatus::Ready)
            return finish(interrupted(), "fallback validation interrupted");
          if (fallback.status == ValidationStatus::Valid) {
            result.validation = fallback;
            result.trajectory = seed.spline;
            result.selected_candidate = CandidateSource::SeedFallback;
            return finish(PlanStatus::Success,
                          "fully validated seed retained after optimizer rejection: " + reason);
          }
        }
        return finish(status, reason);
      };
      Bspline current = seed.spline;
      std::vector<ProxyConstraint> constraints;
      std::vector<int> forced;
      bool retry_roi = false;
      bool refine = false;
      int time_adjustments = 0;
      for (int round = 0; round <= config.max_restarts + config.max_time_adjustments; ++round) {
        if (budget.status() != WorkStatus::Ready)
          return finish(interrupted(), "planning interrupted");
        time = Clock::now();
        const auto built =
            CollisionConstraintBuilder{}.build(view, current, config, budget, forced);
        result.timings.constraints_ms += Elapsed(time);
        result.astar_calls += built.astar_calls;
        result.collision_segments.insert(result.collision_segments.end(), built.segments.begin(),
                                         built.segments.end());
        if (budget.status() != WorkStatus::Ready) return finish(interrupted(), built.reason);
        if (built.status == ConstraintStatus::SearchFailed) {
          if (attempt + 1 < attempts) {
            retry_roi = true;
            break;
          }
          return finish(PlanStatus::NoPath,
                        built.reason + " (bounded ROI; not a global impossibility proof)");
        }
        if (built.status == ConstraintStatus::ResourceLimit)
          return finish(PlanStatus::ResourceLimit, built.reason);
        if (built.status != ConstraintStatus::NoCollision &&
            built.status != ConstraintStatus::ConstraintsReady)
          return finish(PlanStatus::ConstraintFailure, built.reason);
        constraints.insert(constraints.end(), built.constraints.begin(), built.constraints.end());
        if (constraints.size() > config.max_control_points * 64)
          return finish(PlanStatus::ResourceLimit, "too many accumulated collision proxies");
        time = Clock::now();
        const Bspline reference = current;
        const auto optimized = ReboundOptimizer{}.optimize(current, constraints, config, budget,
                                                           refine ? &reference : nullptr);
        result.timings.optimization_ms += Elapsed(time);
        result.optimizer_status = optimized.status;
        result.solver_code = optimized.solver_code;
        result.evaluations += optimized.evaluations;
        result.iterations += optimized.iterations;
        result.smoothness_cost = optimized.cost.smoothness;
        result.collision_cost = optimized.cost.collision;
        result.feasibility_cost = optimized.cost.feasibility;
        result.fitness_cost = optimized.cost.fitness;
        if (result.evaluations == optimized.evaluations)
          result.initial_cost = optimized.initial_cost;
        result.final_cost = optimized.final_cost;
        if (budget.status() != WorkStatus::Ready)
          return finish(interrupted(), "optimizer interrupted");
        if (!optimized.spline)
          return reject(PlanStatus::OptimizerFailure, toString(optimized.status));
        current = *optimized.spline;
        time = Clock::now();
        result.validation = TrajectoryValidator{}.validate(view, current, request, config, budget);
        result.timings.validation_ms += Elapsed(time);
        result.constraints = constraints;
        if (budget.status() != WorkStatus::Ready)
          return finish(interrupted(), "validation interrupted");
        if (result.validation.status == ValidationStatus::Valid) {
          result.trajectory = current;
          result.selected_candidate = CandidateSource::Optimized;
          return finish(
              PlanStatus::Success,
              "candidate verified on frozen snapshot; live use requires latest-map revalidation");
        }
        if (result.validation.status == ValidationStatus::DynamicsExceeded &&
            time_adjustments < config.max_time_adjustments) {
          ++time_adjustments;
          ++result.time_adjustments;
          const double scale =
              1.05 *
              std::max({1.0, result.validation.speed_bound / config.max_velocity,
                        std::sqrt(result.validation.acceleration_bound / config.max_acceleration)});
          const double interval = uniformInterval(current) * scale;
          if (current.duration() * scale > config.max_duration)
            return reject(PlanStatus::ValidationFailure,
                          "required duration exceeds experiment limit");
          Eigen::MatrixXd points = current.controlPoints();
          // 加时后重建运动边界，避免把实测v/a按1/r、1/r²悄悄缩小。
          enforceBoundary(points, interval, request.start, request.goal);
          current = Bspline(std::move(points), 3, interval);
          forced.clear();
          refine = true;
          continue;
        }
        if (result.validation.status == ValidationStatus::Collision &&
            result.restarts < config.max_restarts) {
          ++result.restarts;
          forced = {result.validation.collision_span};
          refine = false;
          continue;
        }
        if (result.validation.status == ValidationStatus::Unverified && attempt + 1 < attempts) {
          retry_roi = true;
          break;
        }
        return reject(PlanStatus::ValidationFailure, toString(result.validation.status));
      }
      if (!retry_roi)
        return reject(PlanStatus::ValidationFailure,
                      "bounded optimization restart count exhausted");
      // view和本轮工作区在此析构；下一次尝试不同时持有两个派生地图掩码。
    }
    return finish(PlanStatus::NoPath, "bounded ROI attempts exhausted");
  } catch (const std::bad_alloc&) {
    return finish(PlanStatus::ResourceLimit, "planning allocation failed");
  } catch (const std::length_error& e) {
    return finish(PlanStatus::ResourceLimit, e.what());
  } catch (const std::exception& e) {
    if (budget.status() != WorkStatus::Ready) return finish(interrupted(), e.what());
    return finish(PlanStatus::InvalidInput, e.what());
  }
}
}  // namespace ego_planner
