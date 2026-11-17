// Mathematical costs adapted from EGO-Planner bspline_optimizer.cpp; GPL-3.0.
#include <ego_planner/rebound_optimizer.h>

#include <algorithm>
#include <cmath>
#include <lbfgs.hpp>
#include <new>
#include <stdexcept>

namespace ego_planner {
CostEvaluation evaluateObjective(const Eigen::MatrixXd& q, double dt,
                                 const std::vector<ProxyConstraint>& constraints,
                                 const PlanningConfig& c, const Eigen::MatrixXd* reference) {
  validateConfig(c);
  if (q.rows() != 3 || q.cols() < 8 || q.cols() > static_cast<Eigen::Index>(c.max_control_points) ||
      !q.allFinite() || !std::isfinite(dt) || dt <= 0 ||
      (reference &&
       (reference->rows() != 3 || reference->cols() != q.cols() || !reference->allFinite())))
    throw std::invalid_argument("invalid objective geometry/time/reference");
  CostEvaluation out;
  out.gradient = Eigen::MatrixXd::Zero(3, q.cols());
  for (Eigen::Index i = 0; i < q.cols() - 3; ++i) {
    const Vec3 jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
    out.smoothness += jerk.squaredNorm();
    for (int k = 0; k < 4; ++k) {
      constexpr double factor[4] = {-1, 3, -3, 1};
      out.gradient.col(i + k) += 2 * c.smoothness_weight * factor[k] * jerk;
    }
  }
  const double margin = c.soft_clearance;
  for (const auto& proxy : constraints) {
    if (proxy.control_index < 3 || proxy.control_index >= q.cols() - 3 ||
        !proxy.base_point.allFinite() || !proxy.direction.allFinite() ||
        std::abs(proxy.direction.norm() - 1.0) > 1e-6)
      throw std::invalid_argument("invalid collision proxy");
    const double error =
        margin - (q.col(proxy.control_index) - proxy.base_point).dot(proxy.direction);
    if (error <= 0) continue;
    double cost, derivative;
    if (error < margin) {
      cost = error * error * error;
      derivative = 3 * error * error;
    } else {
      cost = 3 * margin * error * error - 3 * margin * margin * error + margin * margin * margin;
      derivative = 6 * margin * error - 3 * margin * margin;
    }
    out.collision += cost;
    out.gradient.col(proxy.control_index) -= c.collision_weight * derivative * proxy.direction;
  }
  const double inv = 1 / dt, inv2 = inv * inv;
  // 保留原EGO逐轴软代价；最终动力学限制另用三维范数硬验证。
  for (Eigen::Index i = 0; i < q.cols() - 1; ++i)
    for (int axis = 0; axis < 3; ++axis) {
      const double velocity = (q(axis, i + 1) - q(axis, i)) * inv;
      const double error = std::abs(velocity) - c.max_velocity;
      if (error > 0) {
        out.feasibility += error * error * inv2;
        const double grad =
            2 * error * std::copysign(1.0, velocity) * inv * inv2 * c.feasibility_weight;
        out.gradient(axis, i) -= grad;
        out.gradient(axis, i + 1) += grad;
      }
    }
  for (Eigen::Index i = 0; i < q.cols() - 2; ++i)
    for (int axis = 0; axis < 3; ++axis) {
      const double acceleration = (q(axis, i + 2) - 2 * q(axis, i + 1) + q(axis, i)) * inv2;
      const double error = std::abs(acceleration) - c.max_acceleration;
      if (error > 0) {
        out.feasibility += error * error;
        const double grad =
            2 * error * std::copysign(1.0, acceleration) * inv2 * c.feasibility_weight;
        out.gradient(axis, i) += grad;
        out.gradient(axis, i + 1) -= 2 * grad;
        out.gradient(axis, i + 2) += grad;
      }
    }
  if (reference)
    for (Eigen::Index i = 2; i < q.cols() - 2; ++i) {
      const Vec3 target =
          (reference->col(i - 1) + 4 * reference->col(i) + reference->col(i + 1)) / 6;
      const Vec3 error = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6 - target;
      Vec3 direction = reference->col(i + 1) - reference->col(i - 1);
      Eigen::Matrix3d weight = Eigen::Matrix3d::Identity();
      if (direction.norm() > 1e-9) {
        direction.normalize();
        weight -= .96 * direction * direction.transpose();
      }
      out.fitness += error.dot(weight * error);
      const Vec3 grad = 2 * c.fitness_weight * weight * error;
      out.gradient.col(i - 1) += grad / 6;
      out.gradient.col(i) += 4 * grad / 6;
      out.gradient.col(i + 1) += grad / 6;
    }
  out.total = c.smoothness_weight * out.smoothness + c.collision_weight * out.collision +
              c.feasibility_weight * out.feasibility + c.fitness_weight * out.fitness;
  if (!std::isfinite(out.total) || !out.gradient.allFinite())
    throw std::overflow_error("nonfinite objective or gradient");
  return out;
}

const char* toString(OptimizerStatus s) {
  switch (s) {
    case OptimizerStatus::Converged:
      return "Converged";
    case OptimizerStatus::AlreadyMinimized:
      return "AlreadyMinimized";
    case OptimizerStatus::MaxIterations:
      return "MaxIterations";
    case OptimizerStatus::NumericalFailure:
      return "NumericalFailure";
    case OptimizerStatus::ResourceLimit:
      return "ResourceLimit";
    case OptimizerStatus::Timeout:
      return "Timeout";
    case OptimizerStatus::Cancelled:
      return "Cancelled";
  }
  return "NumericalFailure";
}
namespace {
struct ObjectiveContext {
  Eigen::MatrixXd points;
  const std::vector<ProxyConstraint>& constraints;
  const PlanningConfig& config;
  const Eigen::MatrixXd* reference;
  double interval;
  WorkBudget budget;
  std::size_t evaluations = 0;
  int iterations = 0;
  bool failed = false;
};
int Progress(void* data, const double*, const double*, double, double, double, double, int, int k,
             int) {
  auto& ctx = *static_cast<ObjectiveContext*>(data);
  ctx.iterations = std::max(ctx.iterations, k);
  return ctx.failed || ctx.budget.status() != WorkStatus::Ready;
}
double Evaluate(void* data, const double* x, double* gradient, int n) {
  auto& ctx = *static_cast<ObjectiveContext*>(data);
  std::fill(gradient, gradient + n, 0.0);
  if (ctx.budget.status() != WorkStatus::Ready || ctx.failed) return 1e100;
  try {
    Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>> values(x, 3, n / 3);
    ctx.points.middleCols(3, n / 3) = values;
    const auto value =
        evaluateObjective(ctx.points, ctx.interval, ctx.constraints, ctx.config, ctx.reference);
    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> mapped(gradient, 3, n / 3);
    mapped = value.gradient.middleCols(3, n / 3);
    ++ctx.evaluations;
    return value.total;
  } catch (...) {
    ctx.failed = true;
    return 1e100;
  }
}
}  // namespace

OptimizerResult ReboundOptimizer::optimize(const Bspline& seed,
                                           const std::vector<ProxyConstraint>& constraints,
                                           const PlanningConfig& config, WorkBudget budget,
                                           const Bspline* refine_reference) const {
  OptimizerResult result;
  auto interrupted = [&]() {
    if (budget.status() == WorkStatus::Ready) return false;
    result.status = budget.status() == WorkStatus::Cancelled ? OptimizerStatus::Cancelled
                                                             : OptimizerStatus::Timeout;
    result.spline.reset();
    return true;
  };
  if (interrupted()) return result;
  validateConfig(config);
  const double dt = uniformInterval(seed);
  const auto* reference = refine_reference ? &refine_reference->controlPoints() : nullptr;
  ObjectiveContext ctx{seed.controlPoints(), constraints, config, reference, dt, budget};
  try {
    result.initial_cost = evaluateObjective(ctx.points, dt, constraints, config, reference).total;
    Eigen::MatrixXd variables = ctx.points.middleCols(3, ctx.points.cols() - 6);
    lbfgs::lbfgs_parameter_t parameters;
    lbfgs::lbfgs_load_default_parameters(&parameters);
    parameters.max_iterations = config.max_iterations;
    parameters.max_linesearch = 30;
    parameters.past = 3;
    parameters.delta = 1e-6;
    result.solver_code =
        lbfgs::lbfgs_optimize(static_cast<int>(variables.size()), variables.data(),
                              &result.final_cost, Evaluate, nullptr, Progress, &ctx, &parameters);
    result.evaluations = ctx.evaluations;
    result.iterations = ctx.iterations;
    if (interrupted()) return result;
    if (ctx.failed) return result;
    if (result.solver_code == lbfgs::LBFGSERR_OUTOFMEMORY) {
      result.status = OptimizerStatus::ResourceLimit;
      return result;
    }
    if (result.solver_code == lbfgs::LBFGS_ALREADY_MINIMIZED)
      result.status = OptimizerStatus::AlreadyMinimized;
    else if (result.solver_code >= 0)
      result.status = OptimizerStatus::Converged;
    else if (result.solver_code == lbfgs::LBFGSERR_MAXIMUMITERATION)
      result.status = OptimizerStatus::MaxIterations;
    else
      return result;
    ctx.points.middleCols(3, variables.cols()) = variables;
    result.cost = evaluateObjective(ctx.points, dt, constraints, config, reference);
    result.final_cost = result.cost.total;
    if (interrupted()) return result;
    result.spline.emplace(std::move(ctx.points), 3, dt);
  } catch (const std::bad_alloc&) {
    result.status = OptimizerStatus::ResourceLimit;
  } catch (const std::exception&) {
    result.status = OptimizerStatus::NumericalFailure;
  }
  return result;
}
}  // namespace ego_planner
