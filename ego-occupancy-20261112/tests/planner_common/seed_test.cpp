#include <ego_planner/seed_builder.h>

#include <cmath>

#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
int main() {
  try {
    PlanningConfig config;
    PlanningRequest request;
    request.start.position = {0, 0, 0};
    request.goal.position = {2, 0, 0};
    auto seed = SeedBuilder{}.build(request, config);
    CHECK((seed.spline.evaluate(0) - request.start.position).norm() < 1e-9);
    CHECK((seed.spline.evaluate(seed.spline.duration()) - request.goal.position).norm() < 1e-9);
    CHECK(seed.spline.derivative().evaluate(0).norm() < 1e-9);
    CHECK(seed.spline.derivative().derivative().evaluate(seed.spline.duration()).norm() < 1e-9);
    request.start.velocity = {.2, .1, 0};
    request.start.acceleration = {.1, -.1, .05};
    request.goal.velocity = {-.1, .1, 0};
    request.goal.acceleration = {.1, 0, -.1};
    request.reference_points = {{.6, .4, .1}, {1.4, .4, .1}};
    seed = SeedBuilder{}.build(request, config);
    const auto v = seed.spline.derivative(), a = v.derivative();
    CHECK((v.evaluate(0) - request.start.velocity).norm() < 1e-9);
    CHECK((v.evaluate(v.duration()) - request.goal.velocity).norm() < 1e-9);
    CHECK((a.evaluate(0) - request.start.acceleration).norm() < 1e-8);
    CHECK((a.evaluate(a.duration()) - request.goal.acceleration).norm() < 1e-8);
    double time = 0;
    for (std::size_t i = 0; i + 1 < seed.reference.segments().size(); ++i) {
      const auto& left = seed.reference.segments()[i];
      const auto& right = seed.reference.segments()[i + 1];
      for (int d = 0; d < 3; ++d)
        CHECK((left.evaluate(left.duration(), d) - right.evaluate(0, d)).norm() < 1e-8);
      time += left.duration();
      CHECK((seed.reference.evaluate(time) - request.reference_points[i]).norm() < 1e-8);
    }
    auto slower = SeedBuilder{}.build(request, config, 2.0);
    CHECK((slower.spline.derivative().evaluate(0) - request.start.velocity).norm() < 1e-9);
    CHECK(
        (slower.spline.derivative().derivative().evaluate(0) - request.start.acceleration).norm() <
        1e-8);
    auto tiny_spacing = config;
    tiny_spacing.control_point_spacing = std::numeric_limits<double>::denorm_min();
    CHECK(SeedBuilder{}.build(request, tiny_spacing).spline.controlPoints().cols() ==
          static_cast<Eigen::Index>(tiny_spacing.max_control_points));
    Reject([&] {
      auto bad = config;
      bad.max_velocity = 0;
      SeedBuilder{}.build(request, bad);
    });
    Reject([&] {
      auto bad = config;
      bad.max_control_points = 6;
      SeedBuilder{}.build(request, bad);
    });
    Reject([&] {
      auto bad = request;
      bad.goal = bad.start;
      bad.reference_points.clear();
      SeedBuilder{}.build(bad, config);
    });
    Reject([&] {
      auto bad = request;
      bad.reference_points = {bad.start.position};
      SeedBuilder{}.build(bad, config);
    });
    std::cout
        << "PASS seed: exact p/v/a, C2 minimum-jerk references, moving retime, invalid inputs\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
