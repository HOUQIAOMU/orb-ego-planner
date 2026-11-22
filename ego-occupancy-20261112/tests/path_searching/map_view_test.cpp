#include "test_support.h"
#include <ego_planner/planning_map_view.h>
#include <ego_planner/astar.h>
#include <limits>
using namespace ego_planner;
using namespace planning_test;
using ego_occupancy::VoxelState;
int main() {
  try {
    auto map=FreeMap();
    PlanningMapView free(map,{{0,0,0},{9,9,9}},Vec3::Zero());
    CHECK(!free.blocked({4,4,4}));
    CHECK(free.blocked({-1,4,4}));
    CHECK(free.snapshot().worldToIndex(Vec3(-0.2,-0.2,-0.2)).value()==Index(0,0,0));
    CHECK(!free.snapshot().worldToIndex(Vec3(-0.226,0,0)));
    auto special=MakeMap({9,9,9},.05,{-.225,-.225,-.225},[](Index i){
      if(i==Index(4,4,4)) return VoxelState::Occupied;
      if(i==Index(7,4,4)) return VoxelState::Unknown;
      return VoxelState::Free;
    });
    PlanningMapView raw(special,{{0,0,0},{9,9,9}},Vec3::Zero());
    CHECK(raw.blocked({4,4,4})); CHECK(raw.blocked({7,4,4}));
    CHECK(!raw.edgeFree({3,4,4},{4,5,4})); // 对角线不能贴着占据格穿角。
    CHECK(!raw.edgeFree({2,2,2},{4,2,2}));
    CHECK(raw.edgeFree({1,1,1},{2,2,2}));
    PlanningMapView body(special,{{0,0,0},{9,9,9}},Vec3(.1,.1,.1));
    CHECK(body.blocked({2,4,4})); CHECK(body.blocked({5,4,4}));
    CHECK(body.blocked({0,0,0}));
    auto inflated=MakeMap({9,9,9},.05,Vec3::Zero(),[](Index){return VoxelState::Free;},true);
    PlanningMapView no_double(inflated,{{0,0,0},{9,9,9}},Vec3::Zero());
    CHECK(!no_double.blocked({4,4,4}));
    Reject([&]{PlanningMapView bad(map,{{0,0,0},{9,9,9}},Vec3(-1,0,0));});
    Reject([&]{PlanningMapView bad(map,{{0,0,0},{10,9,9}},Vec3::Zero());});
    Reject([&]{PlanningMapView bad(map,{{0,0,0},{9,9,9}},Vec3::Zero(),{},10);});
    Reject([&]{PlanningMapView bad(nullptr,{{0,0,0},{1,1,1}},Vec3::Zero());});
    std::atomic<bool> stop{true};
    Reject([&]{PlanningMapView bad(map,{{0,0,0},{9,9,9}},Vec3::Zero(),{Clock::now()+std::chrono::seconds(1),&stop});});
    Reject([&]{PlanningMapView bad(map,{{0,0,0},{9,9,9}},Vec3::Zero(),{Clock::now(),nullptr});});
    // 当前8×4×8m、5cm地图为2,048,000格，必须覆盖真实布局而非只测小夹具。
    auto full_map=MakeMap({160,80,160},.05,Vec3(-4,-2,-4),[](Index){return VoxelState::Free;});
    PlanningMapView full(full_map,{{0,0,0},{160,80,160}},Vec3(.1,.1,.1),
                         {Clock::now()+std::chrono::seconds(10),nullptr});
    CHECK(!full.blocked({80,40,80}));
    CHECK(full.blocked({0,40,80}));
    const auto full_path=Astar{}.search(full,full.snapshot().indexToWorld({80,40,80}),
      full.snapshot().indexToWorld({82,40,80}),{},
      {Clock::now()+std::chrono::seconds(10),nullptr});
    CHECK(full_path.status==SearchStatus::Success);
    CHECK(std::abs(full_path.length_m-.1)<1e-10);
    std::cout<<"PASS map view: tri-state, clearance, ROI, corners, budgets\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
