#include <ego_planner/planning_fsm.h>

#include "test_support.h"
using namespace ego_planner;
using namespace planning_test;
int main() {
  try {
    PlanningFsm fsm;
    ego_occupancy::Metadata map;
    map.source = {1, 2, 3};
    map.epoch = 4;
    map.sequence = 1;
    CHECK(fsm.state() == PlanningState::WaitMap);
    fsm.onEvent({EventType::MapAvailable, map, 0, 0});
    CHECK(fsm.state() == PlanningState::WaitState);
    fsm.onEvent({EventType::StateValid, map, 0, 0});
    CHECK(fsm.state() == PlanningState::WaitTask);
    auto action = fsm.onEvent({EventType::TaskSet, map, 1, 1});
    CHECK(action.solve && fsm.state() == PlanningState::ReplanPending);
    fsm.onEvent({EventType::SolveStarted, map, 1, 1});
    fsm.onEvent({EventType::SolveSucceeded, map, 1, 1});
    CHECK(!fsm.ready() && fsm.state() == PlanningState::Validating);
    fsm.onEvent({EventType::ValidationPassed, map, 1, 1});
    CHECK(fsm.ready());
    ++map.sequence;
    CHECK(fsm.onEvent({EventType::MapAvailable, map, 0, 0}).validate);
    CHECK(!fsm.ready());
    auto old = map;
    --old.sequence;
    fsm.onEvent({EventType::ValidationPassed, old, 1, 1});
    CHECK(!fsm.ready());
    fsm.onEvent({EventType::ValidationPassed, map, 1, 1});
    CHECK(fsm.ready());
    fsm.onEvent({EventType::TaskSet, map, 2, 2});
    fsm.onEvent({EventType::ValidationPassed, map, 1, 1});
    CHECK(!fsm.ready());
    ++map.epoch;
    action = fsm.onEvent({EventType::MapAvailable, map, 0, 0});
    CHECK(action.cancel && !fsm.ready() && fsm.state() == PlanningState::WaitState);
    const auto prior_epoch = old;
    fsm.onEvent({EventType::StateValid, prior_epoch, 0, 0});
    CHECK(fsm.state() == PlanningState::WaitState);
    CHECK(!fsm.onEvent({EventType::TaskSet, prior_epoch, 99, 99}).solve);
    CHECK(!fsm.onEvent({EventType::TaskSet, map, 3, 3}).solve);
    CHECK(fsm.state() == PlanningState::WaitState);
    CHECK(fsm.onEvent({EventType::StateValid, map, 0, 0}).solve);
    fsm.onEvent({EventType::TaskSet, map, 3, 3});
    fsm.onEvent({EventType::StateInvalid, map, 0, 0});
    CHECK(!fsm.ready() && fsm.state() == PlanningState::WaitState);
    fsm.onEvent({EventType::StateValid, map, 0, 0});
    CHECK(fsm.state() == PlanningState::WaitTask);
    std::cout << "PASS FSM: latest-version validation, late result rejection, epoch reset, state "
                 "invalidation\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
