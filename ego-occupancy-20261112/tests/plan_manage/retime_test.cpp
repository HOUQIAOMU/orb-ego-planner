#include <ego_planner/planner_core.h>

#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
int main() {
  try {
    auto map = MakeMap({70, 70, 40}, .1, {-1, -2, -1},
                       [](Index) { return ego_occupancy::VoxelState::Free; });
    PlanningConfig config;
    config.half_extent.setZero();
    config.max_acceleration = .05;
    config.roi_margin = 1.0;
    PlanningRequest request;
    request.start.position = {.25, .85, .55};
    request.goal.position = {2.15, .85, .55};
    request.start.velocity = {.1, .03, 0};
    request.goal.velocity = {.05, 0, 0};
    request.start.acceleration = {.01, 0, 0};
    request.goal.acceleration = {0, .005, 0};
    const auto result = PlannerCore{}.plan(map, request, config,
                                           {Clock::now() + std::chrono::seconds(10), nullptr});
    std::cerr << "retime: " << toString(result.status) << " " << result.reason
              << " adjustments=" << result.time_adjustments << '\n';
    CHECK(result.status == PlanStatus::Success && result.trajectory);
    CHECK(result.time_adjustments > 0);
    const auto& curve = *result.trajectory;
    const auto velocity = curve.derivative(), acceleration = velocity.derivative();
    CHECK((velocity.evaluate(0) - request.start.velocity).norm() < 1e-8);
    CHECK((velocity.evaluate(curve.duration()) - request.goal.velocity).norm() < 1e-8);
    CHECK((acceleration.evaluate(0) - request.start.acceleration).norm() < 1e-8);
    CHECK((acceleration.evaluate(curve.duration()) - request.goal.acceleration).norm() < 1e-8);
    CHECK(result.validation.acceleration_bound <= config.max_acceleration * (1 + 1e-9));
    std::cout << "PASS retime: real PlannerCore time adjustment preserves nonzero endpoint p/v/a\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
