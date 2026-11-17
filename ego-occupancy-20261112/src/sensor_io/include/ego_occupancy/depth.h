#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <ego_occupancy/map.h>

namespace ego_occupancy {

struct CameraModel {
  int width = 0;
  int height = 0;
  int stride = 2;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double depth_scale = 0.0;
  double min_depth = 0.2;
  double max_depth = 8.0;
  std::string distortion_model = "rectified";
  std::array<double, 5> distortion{};
};

class DepthProjector {
 public:
  explicit DepthProjector(const CameraModel& camera);

  RayObservation project(const cv::Mat& depth,
                         const Eigen::Isometry3d& T_grid_camera,
                         SourceStamp source,
                         std::uint64_t frame_index,
                         double timestamp_sec,
                         TimePoint received) const;

  const CameraModel& camera() const { return camera_; }

 private:
  CameraModel camera_;
  std::vector<Vec3> rays_;
};

CameraModel readCameraModel(const std::string& yaml, int stride = 2);
Config readMapConfig(const std::string& yaml);

}  // namespace ego_occupancy
