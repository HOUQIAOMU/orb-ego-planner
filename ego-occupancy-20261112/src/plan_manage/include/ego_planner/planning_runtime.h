#pragma once

#include <ego_planner/planner_core.h>
#include <ego_planner/planning_fsm.h>

#include <Eigen/Geometry>
#include <functional>
#include <memory>

namespace ego_planner {

struct RuntimeState {
  ego_occupancy::SourceStamp source;
  Eigen::Isometry3d T_world_camera = Eigen::Isometry3d::Identity();
  Clock::time_point received{};  // 主机steady_clock，不能填传感器时间戳。
  std::uint64_t state_id = 0;
  bool valid = false;
};
struct RuntimeOptions {
  double max_state_age_sec = 0.5;
  int budget_ms = 200;
  int retry_interval_ms = 500;
};
enum class TrajectoryValidity { Invalid, PendingValidation, Ready };
const char* toString(TrajectoryValidity validity);
struct RuntimeCandidate {
  std::uint64_t candidate_id = 0;
  std::uint64_t frozen_state_id = 0;
  PlanningRequest request;
  PlanResult plan;
  ego_occupancy::Metadata validated_on;
  ValidationReport latest_validation;
};
struct RuntimeView {
  PlanningState state = PlanningState::WaitMap;
  TrajectoryValidity validity = TrajectoryValidity::Invalid;
  std::string reason = "waiting for map/state/task";
  std::uint64_t request_id = 0;
  std::uint64_t frozen_state_id = 0;
  ego_occupancy::Metadata observed_map;
  double state_age_ms = -1;
  double map_age_ms = -1;
  double pending_ms = 0;
  double last_validation_ms = 0;
  bool has_task = false;
  std::optional<PlanningRequest> frozen_request;
  std::shared_ptr<const RuntimeCandidate> candidate;
  std::size_t solve_count = 0;
  std::size_t revalidation_count = 0;
  std::size_t discarded_count = 0;
};
struct RuntimeEvent {
  std::string kind;
  RuntimeView view;
  Clock::time_point time;
};

/// 单worker、有界最新输入；算法与IO回调均不在状态锁内执行。
/// Ready仅对view()读取瞬间的最新地图/状态成立，不是控制执行许可。
class PlanningRuntime {
 public:
  using EventSink = std::function<void(const RuntimeEvent&)>;
  PlanningRuntime(ego_occupancy::MapReader reader, PlanningConfig config,
                  RuntimeOptions options = {}, EventSink sink = {});
  ~PlanningRuntime();
  PlanningRuntime(const PlanningRuntime&) = delete;
  PlanningRuntime& operator=(const PlanningRuntime&) = delete;
  void updateState(const RuntimeState& state);
  bool submit(PlanningRequest request);
  /// 相机光轴前方短段，只作静止几何探针，起点不是相机原点。
  bool triggerProbe(double start_depth_m = 0.5, double goal_depth_m = 0.9);
  void clear();
  RuntimeView view() const;
  void stop();
  std::string error() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace ego_planner
