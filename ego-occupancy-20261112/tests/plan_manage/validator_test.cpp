#include <ego_planner/seed_builder.h>
#include <ego_planner/trajectory_validator.h>

#include <iomanip>

#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
WorkBudget Budget() { return {Clock::now() + std::chrono::seconds(10), nullptr}; }
int main() {
  try {
    PlanningConfig c;
    c.half_extent.setZero();
    c.max_velocity = 5;
    c.max_acceleration = 10;
    PlanningRequest r;
    r.start.position = {.25, .85, .55};
    r.goal.position = {2.15, .85, .55};
    auto seed = SeedBuilder{}.build(r, c);
    Roi roi{Index::Zero(), Index(25, 21, 13)};
    auto make = [&](int wall, ego_occupancy::VoxelState state) {
      return MakeMap(roi.maximum_exclusive, .1, Vec3::Zero(), [=](Index p) {
        return p.x() == wall ? state : ego_occupancy::VoxelState::Free;
      });
    };
    PlanningMapView free(make(-1, ego_occupancy::VoxelState::Free), roi, Vec3::Zero(), Budget());
    auto valid = TrajectoryValidator{}.validate(free, seed.spline, r, c, Budget());
    CHECK(valid.status == ValidationStatus::Valid);
    // 末1/3薄壁，不能沿用上游只检查前2/3的截断行为。
    PlanningMapView wall(make(19, ego_occupancy::VoxelState::Occupied), roi, Vec3::Zero(),
                         Budget());
    auto collision = TrajectoryValidator{}.validate(wall, seed.spline, r, c, Budget());
    CHECK(collision.status == ValidationStatus::Collision && collision.collision_span >= 0);
    PlanningMapView unknown(make(12, ego_occupancy::VoxelState::Unknown), roi, Vec3::Zero(),
                            Budget());
    const auto unknown_report =
        TrajectoryValidator{}.validate(unknown, seed.spline, r, c, Budget());
    std::cerr << "unknown: " << toString(unknown_report.status)
              << " visits=" << unknown_report.visits << '\n';
    CHECK(unknown_report.status == ValidationStatus::Collision);
    auto low = c;
    low.max_validation_visits = 1;
    CHECK(TrajectoryValidator{}.validate(free, seed.spline, r, low, Budget()).status ==
          ValidationStatus::Unverified);
    auto fast = seed.spline.scaledTime(.01);
    CHECK(TrajectoryValidator{}.validate(free, fast, r, c, Budget()).status ==
          ValidationStatus::DynamicsExceeded);
    auto changed = r;
    changed.start.position.x() += .01;
    CHECK(TrajectoryValidator{}.validate(free, seed.spline, changed, c, Budget()).status ==
          ValidationStatus::BoundaryMismatch);
    r.gate = Gate{};
    r.gate->center = {1.25, .85, .55};
    r.gate->half_width = .3;
    r.gate->half_height = .3;
    CHECK(TrajectoryValidator{}.validate(free, seed.spline, r, c, Budget()).status ==
          ValidationStatus::Valid);
    r.gate->center.y() += .5;
    CHECK(TrajectoryValidator{}.validate(free, seed.spline, r, c, Budget()).status ==
          ValidationStatus::GateNotSatisfied);
    r.gate->center.y() -= .5;
    r.gate->normal = -Vec3::UnitX();
    CHECK(TrajectoryValidator{}.validate(free, seed.spline, r, c, Budget()).status ==
          ValidationStatus::GateNotSatisfied);
    // 零尺寸曲线沿体素公共面行进，也要检查闭合接触的另一侧。
    auto contact_r = r;
    contact_r.gate.reset();
    contact_r.start.position.y() = 1.2;
    contact_r.goal.position.y() = 1.2;
    auto contact_seed = SeedBuilder{}.build(contact_r, c);
    auto contact_map = MakeMap(roi.maximum_exclusive, .1, Vec3::Zero(), [](Index p) {
      return p.x() == 12 && p.y() == 11 ? ego_occupancy::VoxelState::Occupied
                                        : ego_occupancy::VoxelState::Free;
    });
    PlanningMapView contact(contact_map, roi, Vec3::Zero(), Budget());
    const auto contact_report =
        TrajectoryValidator{}.validate(contact, contact_seed.spline, contact_r, c, Budget());
    std::cerr << std::setprecision(17) << "contact status=" << toString(contact_report.status)
              << " visits=" << contact_report.visits
              << " y=" << contact_seed.spline.evaluate(contact_seed.spline.duration() / 2).y()
              << " grid="
              << contact_seed.spline.evaluate(contact_seed.spline.duration() / 2).y() / .1 << '\n';
    CHECK(contact_report.status != ValidationStatus::Valid);
    auto mismatched_geometry = c;
    mismatched_geometry.half_extent = Vec3::Constant(.5);
    CHECK(TrajectoryValidator{}
              .validate(free, seed.spline, r, mismatched_geometry, Budget())
              .status == ValidationStatus::InvalidInput);
    auto invalid = r;
    invalid.gate.reset();
    invalid.start.position.x() = std::numeric_limits<double>::quiet_NaN();
    CHECK(TrajectoryValidator{}.validate(free, seed.spline, invalid, c, Budget()).status ==
          ValidationStatus::InvalidInput);
    r.gate->normal = Vec3::UnitX();
    Eigen::MatrixXd wave = Eigen::MatrixXd::Zero(3, 12);
    const double xs[12] = {.25, .25, .25, 1.25, 1.25, 1.25, .6, .6, .6, 2.15, 2.15, 2.15};
    for (int i = 0; i < 12; ++i) wave.col(i) = Vec3(xs[i], .85, .55);
    const Bspline tangent(wave, 3, 2.0);
    CHECK(TrajectoryValidator{}.validate(free, tangent, r, c, Budget()).status ==
          ValidationStatus::GateNotSatisfied);
    wave(0, 3) = 1.9;
    wave(0, 4) = 1.9;
    wave(0, 5) = 1.9;
    const Bspline multiple(wave, 3, 2.0);
    CHECK(TrajectoryValidator{}.validate(free, multiple, r, c, Budget()).status ==
          ValidationStatus::GateNotSatisfied);
    // 独立审查复现：近似均匀节点不能用另一条严格均匀曲线作安全证明。
    Eigen::MatrixXd perturbed_points(3, 8);
    for (int i = 0; i < 8; ++i) perturbed_points.col(i) = Vec3(.25 + .2 * i, .55, .55);
    perturbed_points(1, 4) = 1.55;
    Eigen::VectorXd perturbed_knots(12);
    for (int i = 0; i < 12; ++i) perturbed_knots[i] = i - 3;
    perturbed_knots[5] += 8e-10;
    perturbed_knots[7] -= 8e-10;
    const Bspline perturbed(perturbed_points, 3, perturbed_knots);
    PlanningRequest perturbed_request;
    const auto pv = perturbed.derivative(), pa = pv.derivative();
    perturbed_request.start = {perturbed.evaluate(0), pv.evaluate(0), pa.evaluate(0)};
    perturbed_request.goal = {perturbed.evaluate(perturbed.duration()), pv.evaluate(pv.duration()),
                              pa.evaluate(pa.duration())};
    const auto precise_map =
        MakeMap({25, 30, 13}, .1, Vec3(0, 1.21666666675 - 1.2, 0), [](Index p) {
          return p.y() >= 12 ? ego_occupancy::VoxelState::Occupied
                             : ego_occupancy::VoxelState::Free;
        });
    PlanningMapView precise_view(precise_map, {Index::Zero(), precise_map->dimensions()},
                                 Vec3::Zero(), Budget());
    CHECK(!precise_view.pointFree(perturbed.evaluate(3)));
    CHECK(TrajectoryValidator{}
              .validate(precise_view, perturbed, perturbed_request, c, Budget())
              .status != ValidationStatus::Valid);
    std::atomic<bool> cancel{true};
    CHECK(TrajectoryValidator{}
              .validate(free, seed.spline, r, c, {Clock::now() + std::chrono::seconds(1), &cancel})
              .status == ValidationStatus::Cancelled);
    std::cout << "PASS validator: whole curve, final third, unknown/contact, budgets, pva, 3D "
                 "dynamics, gate\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
