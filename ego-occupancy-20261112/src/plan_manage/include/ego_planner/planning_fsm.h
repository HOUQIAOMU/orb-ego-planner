#pragma once

#include <ego_planner/planning_types.h>

namespace ego_planner {

enum class PlanningState {
  WaitMap,
  WaitState,
  WaitTask,
  ReplanPending,
  Planning,
  Validating,
  Ready,
  Failed
};
const char* toString(PlanningState state);
enum class EventType {
  MapAvailable,
  MapInvalid,
  StateValid,
  StateInvalid,
  TaskSet,
  TaskCleared,
  SolveStarted,
  SolveSucceeded,
  SolveFailed,
  ValidationPassed,
  ValidationFailed,
  Expired
};
struct PlanningEvent {
  EventType type;
  ego_occupancy::Metadata map;
  std::uint64_t request_id = 0;
  std::uint64_t goal_version = 0;
};
struct PlanningActions {
  bool cancel = false;
  bool invalidate = false;
  bool solve = false;
  bool validate = false;
  bool publish = false;
};

/// 纯事件状态机，不持快照/线程/SDK，不按旧曲线假定相机已执行。
class PlanningFsm {
 public:
  /// TaskSet的request_id在本FSM生命周期严格递增；所有状态/任务事件绑定source/epoch。
  PlanningActions onEvent(const PlanningEvent& event);
  PlanningState state() const { return state_; }
  bool ready() const { return ready_; }

 private:
  void waitingState();
  bool matches(const PlanningEvent& event) const;
  PlanningState state_ = PlanningState::WaitMap;
  ego_occupancy::Metadata map_;
  bool map_valid_ = false;
  bool state_valid_ = false;
  bool task_valid_ = false;
  bool ready_ = false;
  std::uint64_t request_id_ = 0;
  std::uint64_t goal_version_ = 0;
};
}  // namespace ego_planner
