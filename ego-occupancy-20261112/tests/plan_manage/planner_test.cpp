#include <ego_planner/planner_core.h>

#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
WorkBudget Budget() { return {Clock::now() + std::chrono::seconds(10), nullptr}; }
int main() {
  try {
    PlanningConfig c;
    c.half_extent.setZero();
    c.roi_margin = 1.5;
    PlanningRequest r;
    r.start.position = {.25, .85, .55};
    r.goal.position = {2.15, .85, .55};
    auto make = [](int mode) {
      return MakeMap({25, 21, 13}, .1, Vec3::Zero(), [=](Index p) {
        return p.x() == 12 && (mode == 2 || (mode == 1 && p.y() < 12))
                   ? ego_occupancy::VoxelState::Occupied
                   : ego_occupancy::VoxelState::Free;
      });
    };
    auto free = PlannerCore{}.plan(make(0), r, c, Budget());
    CHECK(free.status == PlanStatus::Success && free.trajectory.has_value());
    CHECK(free.validation.status == ValidationStatus::Valid && free.astar_calls == 0);
    auto wall = PlannerCore{}.plan(make(1), r, c, Budget());
    std::cerr << "wall: " << toString(wall.status) << " " << wall.reason << '\n';
    std::cerr << "restarts=" << wall.restarts << " evals=" << wall.evaluations
              << " cost=" << wall.initial_cost << " -> " << wall.final_cost
              << " collision_span=" << wall.validation.collision_span
              << " visits=" << wall.validation.visits << '\n';
    if (wall.status != PlanStatus::Success)
      for (const auto& proxy : wall.constraints)
        std::cerr << "proxy " << proxy.control_index << " base " << proxy.base_point.transpose()
                  << " dir " << proxy.direction.transpose() << '\n';
    CHECK(wall.status == PlanStatus::Success && wall.trajectory.has_value());
    CHECK(wall.astar_calls > 0 && wall.validation.status == ValidationStatus::Valid);
    auto closed = PlannerCore{}.plan(make(2), r, c, Budget());
    CHECK(closed.status != PlanStatus::Success && !closed.trajectory);
    r.gate = Gate{};
    r.gate->center = {1.25, .85, .55};
    r.gate->half_width = .4;
    r.gate->half_height = .4;
    r.reference_points = {{.95, .85, .55}, {1.55, .85, .55}};
    auto gate = PlannerCore{}.plan(make(0), r, c, Budget());
    CHECK(gate.status == PlanStatus::Success && gate.validation.gate_crossings == 1);
    PlanningRequest offset;
    offset.start.position = {.25, .85, .55};
    offset.goal.position = {2.15, .85, .55};
    offset.gate = Gate{};
    offset.gate->center = {1.25, 1.4, .55};
    offset.gate->half_width = .2;
    offset.gate->half_height = .3;
    offset.reference_points = {{.95, 1.4, .55}, {1.55, 1.4, .55}};
    auto offset_map = make(0);
    const auto offset_seed = SeedBuilder{}.build(offset, c);
    PlanningMapView offset_view(offset_map, {Index::Zero(), offset_map->dimensions()},
                                c.half_extent, Budget());
    CHECK(TrajectoryValidator{}
              .validate(offset_view, offset_seed.spline, offset, c, Budget())
              .status == ValidationStatus::Valid);
    const auto retained = PlannerCore{}.plan(offset_map, offset, c, Budget());
    CHECK(retained.status == PlanStatus::Success && retained.validation.gate_crossings == 1);
    auto old = PlannerCore{}.plan(make(0), r, c, {Clock::now() - std::chrono::seconds(1), nullptr});
    CHECK(old.status == PlanStatus::Timeout && !old.trajectory);
    std::cout
        << "PASS planner: free, rebound around wall, closed rejection, manual gate, timeout\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
