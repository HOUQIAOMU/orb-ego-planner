#pragma once

#include <ego_planner/collision_constraints.h>

#include <optional>

namespace ego_planner {

struct CostEvaluation {
  double total = 0;
  double smoothness = 0;
  double collision = 0;
  double feasibility = 0;
  double fitness = 0;
  Eigen::MatrixXd gradient;
};
/// 固定代理约束下的纯函数，供求解器和有限差分测试共同使用。
CostEvaluation evaluateObjective(const Eigen::MatrixXd& points, double interval,
                                 const std::vector<ProxyConstraint>& constraints,
                                 const PlanningConfig& config,
                                 const Eigen::MatrixXd* reference = nullptr);

enum class OptimizerStatus {
  Converged,
  AlreadyMinimized,
  MaxIterations,
  NumericalFailure,
  ResourceLimit,
  Timeout,
  Cancelled
};
const char* toString(OptimizerStatus status);
struct OptimizerResult {
  OptimizerStatus status = OptimizerStatus::NumericalFailure;
  std::optional<Bspline> spline;
  int solver_code = 0;
  std::size_t evaluations = 0;
  int iterations = 0;
  double initial_cost = 0;
  double final_cost = 0;
  CostEvaluation cost;
};
class ReboundOptimizer {
 public:
  OptimizerResult optimize(const Bspline& seed, const std::vector<ProxyConstraint>& constraints,
                           const PlanningConfig& config, WorkBudget budget,
                           const Bspline* refine_reference = nullptr) const;
};
}  // namespace ego_planner
