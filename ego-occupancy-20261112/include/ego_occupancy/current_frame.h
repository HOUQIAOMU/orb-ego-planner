#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include "pose_bridge.h"
#include <ego_occupancy/depth.h>

namespace ego_occupancy {

struct CurrentFrame {
  /// Owned BGR image. It is immutable after publication and never aliases a
  /// capture buffer.
  cv::Mat bgr;
  ego_pose_bridge::Record pose;
  std::uint64_t capture_index = 0;
  TimePoint received{};
  bool pose_verified = false;
};

struct CurrentFrameSample {
  std::shared_ptr<const CurrentFrame> frame;
  TimePoint sampled_at;
};

class CurrentFrameBuffer {
 public:
  explicit CurrentFrameBuffer(const CameraModel& camera);

  void publish(const cv::Mat& bgr,
               const ego_pose_bridge::Record& pose,
               std::uint64_t capture_index,
               TimePoint received,
               bool verified);

  std::shared_ptr<const CurrentFrame> latest() const;
  CurrentFrameSample sample() const;

 private:
  int width_ = 0;
  int height_ = 0;
  mutable std::mutex mutex_;
  std::shared_ptr<const CurrentFrame> latest_;
};

/// A current pose may be newer than the map, but must share its coordinate epoch.
std::optional<Eigen::Isometry3d> cameraPoseForMap(const CurrentFrame& frame,
                                                  const MapSnapshot& map,
                                                  const ReaderStatus& status,
                                                  TimePoint now,
                                                  double max_age_sec);

/// Center, then top-left/top-right/bottom-right/bottom-left at optical Z=depth_m.
std::array<Vec3, 5> cameraFrustum(const Eigen::Isometry3d& T_world_camera,
                                  const CameraModel& camera,
                                  double depth_m);

}  // namespace ego_occupancy
