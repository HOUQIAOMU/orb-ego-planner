#include <ego_planner/collision_constraints.h>
#include <ego_planner/rebound_optimizer.h>
#include <ego_planner/seed_builder.h>

#include <atomic>
#include <thread>

#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
WorkBudget LongBudget() { return {Clock::now() + std::chrono::seconds(10), nullptr}; }
int main() {
  try {
    PlanningConfig config;
    config.half_extent.setZero();
    PlanningRequest request;
    request.start.position = {.25, .85, .55};
    request.goal.position = {2.15, .85, .55};
    auto seed = SeedBuilder{}.build(request, config);
    const Roi roi{Index::Zero(), Index(25, 21, 13)};
    const auto wall = MakeMap(roi.maximum_exclusive, .1, Vec3::Zero(), [](Index p) {
      return p.x() == 12 && p.y() < 12 ? ego_occupancy::VoxelState::Occupied
                                       : ego_occupancy::VoxelState::Free;
    });
    PlanningMapView view(wall, roi, Vec3::Zero(), LongBudget());
    auto constraints = CollisionConstraintBuilder{}.build(view, seed.spline, config, LongBudget());
    CHECK(constraints.status == ConstraintStatus::ConstraintsReady);
    CHECK(constraints.astar_calls > 0 && !constraints.constraints.empty());
    bool spans_thin_wall = false;
    for (const auto& c : constraints.constraints) {
      CHECK(c.direction.allFinite() && std::abs(c.direction.norm() - 1) < 1e-10);
      if (c.base_point.y() > 1.19 && c.direction.y() > .9) spans_thin_wall = true;
    }
    CHECK(spans_thin_wall);
    auto q = seed.spline.controlPoints();
    q(1, 4) += .11;
    q(2, 5) -= .07;
    // 同时激活逐轴可行性项，测试完整固定代理目标的解析梯度。
    auto tight = config;
    tight.max_velocity = .05;
    tight.nominal_speed = .03;
    tight.max_acceleration = .03;
    const auto value = evaluateObjective(q, seed.interval, constraints.constraints, tight,
                                         &seed.spline.controlPoints());
    for (Eigen::Index i = 0; i < q.cols(); ++i)
      for (int axis = 0; axis < 3; ++axis) {
        auto plus = q, minus = q;
        plus(axis, i) += 1e-6;
        minus(axis, i) -= 1e-6;
        const double numeric = (evaluateObjective(plus, seed.interval, constraints.constraints,
                                                  tight, &seed.spline.controlPoints())
                                    .total -
                                evaluateObjective(minus, seed.interval, constraints.constraints,
                                                  tight, &seed.spline.controlPoints())
                                    .total) /
                               2e-6;
        CHECK(std::abs(numeric - value.gradient(axis, i)) <
              1e-4 * std::max(1.0, std::abs(numeric)));
      }
    auto result =
        ReboundOptimizer{}.optimize(seed.spline, constraints.constraints, config, LongBudget());
    CHECK(result.spline.has_value());
    CHECK(result.final_cost <= result.initial_cost + 1e-8);
    CHECK((result.spline->controlPoints().leftCols(3) - seed.spline.controlPoints().leftCols(3))
              .norm() == 0);
    CHECK((result.spline->controlPoints().rightCols(3) - seed.spline.controlPoints().rightCols(3))
              .norm() == 0);
    std::atomic<bool> cancel{true};
    auto stopped = ReboundOptimizer{}.optimize(seed.spline, {}, config,
                                               {Clock::now() + std::chrono::seconds(1), &cancel});
    CHECK(stopped.status == OptimizerStatus::Cancelled && !stopped.spline);
    auto timeout = ReboundOptimizer{}.optimize(seed.spline, {}, config,
                                               {Clock::now() - std::chrono::seconds(1), nullptr});
    CHECK(timeout.status == OptimizerStatus::Timeout && !timeout.spline);
    auto cancelled = CollisionConstraintBuilder{}.build(
        view, seed.spline, config, {Clock::now() + std::chrono::seconds(1), &cancel});
    CHECK(cancelled.status == ConstraintStatus::Cancelled);
    const auto closed = MakeMap(roi.maximum_exclusive, .1, Vec3::Zero(), [](Index p) {
      return p.x() == 12 ? ego_occupancy::VoxelState::Occupied : ego_occupancy::VoxelState::Free;
    });
    PlanningMapView closed_view(closed, roi, Vec3::Zero(), LongBudget());
    CHECK(
        CollisionConstraintBuilder{}.build(closed_view, seed.spline, config, LongBudget()).status ==
        ConstraintStatus::SearchFailed);
    auto free = MakeMap(roi.maximum_exclusive, .1, Vec3::Zero(),
                        [](Index) { return ego_occupancy::VoxelState::Free; });
    PlanningMapView free_view(free, roi, Vec3::Zero(), LongBudget());
    CHECK(CollisionConstraintBuilder{}.build(free_view, seed.spline, config, LongBudget()).status ==
          ConstraintStatus::NoCollision);
    Reject([&] {
      auto bad = constraints.constraints;
      bad.front().direction.setZero();
      evaluateObjective(q, seed.interval, bad, config);
    });
    std::cout << "PASS optimizer: A* collision constraints, finite-difference gradients, fixed "
                 "endpoints, cancellation, closed wall\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
