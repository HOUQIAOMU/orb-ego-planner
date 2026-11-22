#pragma once
#include <ego_planner/planning_runtime.h>

#include <cmath>

#include "pose_bridge.h"

namespace ego_planner::app {
/// 只在应用边界接触ORB桥接类型；运行时不依赖SDK/OpenCV。
inline RuntimeState cameraState(const ego_pose_bridge::Record& pose, std::uint64_t capture_id,
                                Clock::time_point received, bool verified) {
  RuntimeState state;
  state.source = {pose.session, pose.map_token, pose.revision};
  state.state_id = capture_id;
  state.received = received;
  state.T_world_camera.matrix() =
      Eigen::Map<const Eigen::Matrix4f>(pose.T_world_camera.data()).cast<double>();
  const auto& t = state.T_world_camera;
  state.valid = verified && pose.captured && ego_pose_bridge::IsCurrent(pose) &&
                t.matrix().allFinite() &&
                t.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), .001) &&
                (t.linear().transpose() * t.linear()).isApprox(Eigen::Matrix3d::Identity(), .001) &&
                std::abs(t.linear().determinant() - 1) < .001;
  return state;
}
}  // namespace ego_planner::app
