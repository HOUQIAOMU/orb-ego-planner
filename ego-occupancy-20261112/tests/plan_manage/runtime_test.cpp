#include <ego_occupancy/io.h>
#include <ego_planner/planning_runtime.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
namespace occ = ego_occupancy;

void Fill(occ::OccupancyMapper& mapper, occ::SourceStamp source, int index, bool hit = false) {
  occ::RayObservation obs;
  obs.source = source;
  obs.frame_index = index;
  obs.timestamp_sec = index * .1;
  obs.received = Clock::now();
  obs.origin = {-.8, 0, 0};
  for (int y = -4; y <= 4; ++y)
    for (int z = -4; z <= 4; ++z) obs.rays.push_back({Vec3(.8, y * .08, z * .08), hit});
  CHECK(mapper.integrate(obs).published);
}
RuntimeState State(occ::SourceStamp source) {
  RuntimeState s;
  s.source = source;
  s.received = Clock::now();
  s.valid = true;
  s.state_id = 7;
  s.T_world_camera = Eigen::Isometry3d::Identity();
  return s;
}
PlanningRequest Request() {
  PlanningRequest r;
  r.start.position = {-.4, 0, 0};
  r.goal.position = {.2, 0, 0};
  return r;
}
template <class F>
void Await(F fn) {
  auto deadline = Clock::now() + std::chrono::seconds(5);
  while (!fn() && Clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  CHECK(fn());
}
int main() {
  try {
    occ::Config mc;
    mc.size = {2, 2, 2};
    mc.resolution = .1;
    mc.max_age_sec = 3;
    mc.inflation.setZero();
    occ::SourceStamp a{1, 2, 3};
    occ::OccupancyMapper mapper(mc);
    mapper.reset(a, Vec3::Zero());
    for (int i = 1; i <= 3; ++i) Fill(mapper, a, i);
    PlanningConfig cfg;
    cfg.half_extent.setZero();
    cfg.roi_margin = .2;
    RuntimeOptions opt;
    opt.max_state_age_sec = 3;
    opt.budget_ms = 200;
    opt.retry_interval_ms = 50;
    std::atomic<bool> hold_once{true}, accepted_entered{false}, release_sink{false};
    PlanningRuntime rt(mapper.reader(), cfg, opt, [&](const RuntimeEvent& e) {
      if (e.kind == "Accepted" && hold_once.exchange(false)) {
        accepted_entered = true;
        const auto deadline = Clock::now() + std::chrono::seconds(1);
        while (!release_sink && Clock::now() < deadline)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    rt.updateState(State(a));
    CHECK(rt.submit(Request()));
    Await([&] { return rt.view().validity == TrajectoryValidity::Ready; });
    auto first = rt.view();
    CHECK(first.candidate && first.candidate->plan.trajectory);
    const auto id = first.request_id;
    CHECK(first.candidate->frozen_state_id == 7);
    // 新图发布后，读取端必须立即撤销旧版本Ready，不能等待worker。
    Await([&] { return accepted_entered.load(); });
    Fill(mapper, a, 4);
    Fill(mapper, a, 5);
    Fill(mapper, a, 6);
    auto current = rt.view();
    CHECK(current.validity == TrajectoryValidity::PendingValidation);
    CHECK(current.candidate->validated_on.sequence == 3);
    release_sink = true;
    Await([&] {
      auto v = rt.view();
      return v.validity == TrajectoryValidity::Ready && v.candidate->validated_on.sequence == 6;
    });
    auto moved = State(a);
    moved.T_world_camera.translation() = Vec3(.1, .2, .3);
    moved.state_id = 8;
    rt.updateState(moved);
    CHECK(rt.view().frozen_request->start.position == Request().start.position);
    CHECK(rt.view().candidate->frozen_state_id == 7);
    CHECK(rt.view().request_id == id);
    CHECK(rt.view().solve_count == 1);
    CHECK(rt.view().revalidation_count >= 2);
    // 有限邮箱中连续新请求只接受最新版本，旧完成不能覆盖。
    CHECK(rt.submit(Request()));
    CHECK(rt.submit(Request()));
    auto latest = rt.view().request_id;
    CHECK(latest > id);
    Await([&] {
      auto v = rt.view();
      return v.validity == TrajectoryValidity::Ready && v.candidate->request.request_id == latest;
    });
    RuntimeState lost = State(a);
    lost.valid = false;
    rt.updateState(lost);
    CHECK(rt.view().validity == TrajectoryValidity::Invalid);
    rt.updateState(State(a));
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    CHECK(rt.view().validity != TrajectoryValidity::Ready);
    CHECK(!rt.view().has_task);
    CHECK(rt.submit(Request()));
    Await([&] { return rt.view().validity == TrajectoryValidity::Ready; });
    mapper.invalidate("test tracking reset");
    CHECK(rt.view().validity == TrajectoryValidity::Invalid);
    mapper.reset({1, 2, 4}, Vec3::Zero());
    Fill(mapper, {1, 2, 4}, 1);
    rt.updateState(State({1, 2, 4}));
    Await([&] { return !rt.view().has_task; });
    CHECK(rt.view().validity != TrajectoryValidity::Ready);
    rt.stop();
    rt.stop();
    // 不持有旧租约：停止后连续发布仍然可用。
    for (int i = 2; i < 8; ++i) Fill(mapper, {1, 2, 4}, i);
    // 主机received年龄门控，不能把传感器时间当成steady_clock。
    RuntimeOptions fast = opt;
    fast.max_state_age_sec = .03;
    PlanningRuntime aged(mapper.reader(), cfg, fast);
    aged.updateState(State({1, 2, 4}));
    CHECK(aged.submit(Request()));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(aged.view().validity != TrajectoryValidity::Ready);
    aged.stop();
    // 新障碍占据终点：必须先撤销，再复核/重算失败，不能继续Ready。
    occ::OccupancyMapper obstacle(mc);
    obstacle.reset(a, Vec3::Zero());
    for (int i = 1; i <= 3; ++i) Fill(obstacle, a, i);
    PlanningRuntime obstructed(obstacle.reader(), cfg, opt);
    obstructed.updateState(State(a));
    CHECK(obstructed.submit(Request()));
    Await([&] { return obstructed.view().validity == TrajectoryValidity::Ready; });
    for (int i = 4; i <= 14; ++i) {
      occ::RayObservation o;
      o.source = a;
      o.frame_index = i;
      o.timestamp_sec = i * .1;
      o.received = Clock::now();
      o.origin = {-.8, 0, 0};
      o.rays = {{{.2, 0, 0}, true}};
      CHECK(obstacle.integrate(o).published);
    }
    CHECK(obstacle.reader().acquire()->query(Vec3(.2, 0, 0)).state == occ::VoxelState::Occupied);
    CHECK(obstructed.view().validity != TrajectoryValidity::Ready);
    Await([&] { return obstructed.view().state == PlanningState::Failed; });
    CHECK(!obstructed.view().candidate);
    obstructed.stop();
    std::cout << "PASS runtime: latest-map gate, frozen state, replacement, loss/reset, freshness, "
                 "release, shutdown\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
