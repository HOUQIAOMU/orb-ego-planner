// 调度边界故障注入：替换同步算法返回/延迟，不替代真实算法集成测试。
#include <ego_planner/planning_runtime.h>

#include <atomic>
#include <iostream>
#include <thread>

#include "test_support.h"
using namespace ego_planner;
namespace occ = ego_occupancy;
namespace {
std::atomic<int> mode{0};
}
namespace ego_planner {
const char* toString(PlanStatus s) { return s == PlanStatus::Timeout ? "Timeout" : "InjectedPlan"; }
const char* toString(ValidationStatus s) {
  return s == ValidationStatus::Timeout      ? "Timeout"
         : s == ValidationStatus::Unverified ? "Unverified"
                                             : "InjectedValidation";
}
PlanResult PlannerCore::plan(std::shared_ptr<const occ::MapSnapshot> map, const PlanningRequest&,
                             const PlanningConfig&, WorkBudget) const {
  if (mode == 5) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  PlanResult r;
  r.status = PlanStatus::Success;
  r.planned_on = map->metadata();
  Eigen::MatrixXd cp(3, 8);
  for (int i = 0; i < 8; ++i) cp.col(i) = Vec3(-.4 + .6 * i / 7., .05, .05);
  r.trajectory = Bspline(cp, 3, .3);
  return r;
}
ValidationReport TrajectoryValidator::validate(const PlanningMapView&, const Bspline&,
                                               const PlanningRequest&, const PlanningConfig&,
                                               WorkBudget) const {
  if (mode == 4) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ValidationReport r;
  r.status = mode == 1   ? ValidationStatus::Timeout
             : mode == 2 ? ValidationStatus::Unverified
                         : ValidationStatus::Valid;
  return r;
}
}  // namespace ego_planner
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("mode required");
    mode = std::stoi(argv[1]);
    occ::Config c;
    c.size = {2, 2, 2};
    c.resolution = .1;
    c.max_age_sec = 5;
    c.inflation.setZero();
    occ::OccupancyMapper mapper(c);
    mapper.reset({1, 2, 3}, Vec3::Zero());
    auto publish = [&](int i) {
      occ::RayObservation o;
      o.source = {1, 2, 3};
      o.frame_index = i;
      o.timestamp_sec = i;
      o.received = Clock::now();
      o.origin = {-.8, .05, .05};
      o.rays = {{{.8, .05, .05}, false}};
      CHECK(mapper.integrate(o).published);
    };
    publish(1);
    PlanningConfig cfg;
    cfg.half_extent.setZero();
    RuntimeOptions opt;
    opt.max_state_age_sec = 5;
    opt.retry_interval_ms = 500;
    opt.budget_ms = mode >= 4 ? 1 : 200;
    PlanningRuntime rt(mapper.reader(), cfg, opt,
                       mode == 3 ? PlanningRuntime::EventSink([](const RuntimeEvent&) {
                         throw std::runtime_error("disk full sentinel");
                       })
                                 : PlanningRuntime::EventSink{});
    RuntimeState state;
    state.valid = true;
    state.source = {1, 2, 3};
    state.received = Clock::now();
    rt.updateState(state);
    PlanningRequest q;
    q.start.position = {-.4, .05, .05};
    q.goal.position = {.2, .05, .05};
    const bool submitted = rt.submit(q);
    CHECK(submitted || mode == 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(350));
    const auto v = rt.view();
    std::cerr << "mode=" << mode << " solves=" << v.solve_count
              << " revalidations=" << v.revalidation_count << " reason=" << v.reason << '\n';
    if (mode == 1 || mode == 2) {
      CHECK(v.solve_count == 1);
      CHECK(v.revalidation_count == 1);
      CHECK(v.validity != TrajectoryValidity::Ready);
      CHECK(v.reason.find(mode == 1 ? "Timeout" : "Unverified") != std::string::npos);
      mode = 0;
      publish(2);
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
      CHECK(rt.view().validity == TrajectoryValidity::Ready);
      rt.clear();
      CHECK(!rt.view().has_task);
    } else if (mode == 3) {
      CHECK(v.state == PlanningState::Failed);
      CHECK(v.reason.find("disk full sentinel") != std::string::npos);
      CHECK(rt.error().find("disk full sentinel") != std::string::npos);
      CHECK(v.validity == TrajectoryValidity::Invalid);
      CHECK(!rt.submit(q));
    } else {
      CHECK(v.validity != TrajectoryValidity::Ready);
      if (mode == 5) CHECK(!v.candidate);
    }
    rt.stop();
    std::cout << "PASS injected scheduling boundary\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
