#include <ego_planner/planning_fsm.h>

namespace ego_planner {
const char* toString(PlanningState state) {
  switch (state) {
    case PlanningState::WaitMap:
      return "WAIT_MAP";
    case PlanningState::WaitState:
      return "WAIT_STATE";
    case PlanningState::WaitTask:
      return "WAIT_TASK";
    case PlanningState::ReplanPending:
      return "REPLAN_PENDING";
    case PlanningState::Planning:
      return "PLANNING";
    case PlanningState::Validating:
      return "VALIDATING";
    case PlanningState::Ready:
      return "READY";
    case PlanningState::Failed:
      return "FAILED";
  }
  return "FAILED";
}
void PlanningFsm::waitingState() {
  if (!map_valid_)
    state_ = PlanningState::WaitMap;
  else if (!state_valid_)
    state_ = PlanningState::WaitState;
  else if (!task_valid_)
    state_ = PlanningState::WaitTask;
  else
    state_ = PlanningState::ReplanPending;
}
bool PlanningFsm::matches(const PlanningEvent& e) const {
  return task_valid_ && map_valid_ && state_valid_ && e.request_id == request_id_ &&
         e.goal_version == goal_version_ && e.map.source == map_.source &&
         e.map.epoch == map_.epoch;
}
PlanningActions PlanningFsm::onEvent(const PlanningEvent& e) {
  PlanningActions action;
  switch (e.type) {
    case EventType::MapAvailable: {
      const bool reset = map_valid_ && (e.map.source != map_.source || e.map.epoch != map_.epoch);
      if (map_valid_ && !reset && e.map.sequence <= map_.sequence) break;
      const bool had_ready = ready_;
      map_ = e.map;
      map_valid_ = true;
      ready_ = false;
      if (reset) {
        task_valid_ = false;
        state_valid_ = false;
        action.cancel = true;
        action.invalidate = true;
        waitingState();
      } else if (had_ready || state_ == PlanningState::Validating) {
        action.invalidate = true;
        action.validate = true;
        state_ = PlanningState::Validating;
      } else if (state_ != PlanningState::Planning) {
        waitingState();
        action.solve = state_ == PlanningState::ReplanPending;
      }
      break;
    }
    case EventType::MapInvalid:
    case EventType::StateInvalid:
    case EventType::Expired:
      if (map_valid_ && (e.map.source != map_.source || e.map.epoch != map_.epoch)) break;
      if (e.type == EventType::MapInvalid) {
        map_valid_ = false;
        state_valid_ = false;
        task_valid_ = false;
      } else {
        state_valid_ = false;
        if (e.type == EventType::StateInvalid) task_valid_ = false;
      }
      ready_ = false;
      action.cancel = true;
      action.invalidate = true;
      waitingState();
      break;
    case EventType::StateValid:
      if (!map_valid_ || e.map.source != map_.source || e.map.epoch != map_.epoch) break;
      state_valid_ = true;
      if (state_ == PlanningState::WaitState || state_ == PlanningState::WaitMap) {
        waitingState();
        action.solve = state_ == PlanningState::ReplanPending;
      }
      break;
    case EventType::TaskSet:
      if (!map_valid_ || e.map.source != map_.source || e.map.epoch != map_.epoch ||
          !e.request_id || !e.goal_version || e.request_id <= request_id_ ||
          e.goal_version < goal_version_)
        break;
      request_id_ = e.request_id;
      goal_version_ = e.goal_version;
      task_valid_ = true;
      ready_ = false;
      action.cancel = true;
      action.invalidate = true;
      waitingState();
      action.solve = state_ == PlanningState::ReplanPending;
      break;
    case EventType::TaskCleared:
      task_valid_ = false;
      ready_ = false;
      action.cancel = true;
      action.invalidate = true;
      waitingState();
      break;
    case EventType::SolveStarted:
      if (matches(e) && state_ == PlanningState::ReplanPending) state_ = PlanningState::Planning;
      break;
    case EventType::SolveSucceeded:
      if (matches(e) && state_ == PlanningState::Planning) {
        ready_ = false;
        state_ = PlanningState::Validating;
        action.validate = true;
      }
      break;
    case EventType::SolveFailed:
      if (matches(e) && state_ == PlanningState::Planning) {
        ready_ = false;
        state_ = PlanningState::Failed;
      }
      break;
    case EventType::ValidationPassed:
      if (matches(e) && e.map.sequence == map_.sequence && state_ == PlanningState::Validating) {
        ready_ = true;
        state_ = PlanningState::Ready;
        action.publish = true;
      }
      break;
    case EventType::ValidationFailed:
      if (matches(e) && e.map.sequence == map_.sequence && state_ == PlanningState::Validating) {
        ready_ = false;
        state_ = PlanningState::ReplanPending;
        action.invalidate = true;
        action.solve = true;
      }
      break;
  }
  return action;
}
}  // namespace ego_planner
