#pragma once
#include <ego_occupancy/planning_layer.h>
#include <ego_planner/planning_runtime.h>

#include <filesystem>
#include <fstream>

#include "pose_bridge.h"

namespace ego_planner::app {
/// 装配运行时、历史文件与显示图层；不修改占据概率或相机线程。
class OnlinePlanningSession {
 public:
  OnlinePlanningSession(ego_occupancy::MapReader reader, PlanningConfig config,
                        const std::filesystem::path& directory, double start_m, double goal_m);
  ~OnlinePlanningSession();
  void update(const ego_pose_bridge::Record& pose, std::uint64_t capture_id,
              Clock::time_point received, bool verified);
  bool trigger();
  void clear();
  ego_occupancy::PlanningLayer layer() const;
  void stop();
  std::string error() const;

 private:
  void recordEvent(const RuntimeEvent& event);
  PlanningConfig config_;
  std::filesystem::path directory_;
  double start_m_, goal_m_;
  std::ofstream events_;
  std::unique_ptr<PlanningRuntime> runtime_;  // 最后构造，先停止后销毁日志。
};
}  // namespace ego_planner::app
