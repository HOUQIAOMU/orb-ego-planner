#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include <ego_occupancy/current_frame.h>
#include <ego_occupancy/display_color.h>

namespace ego_occupancy {

struct DisplaySettings {
  ColorMode color_mode = ColorMode::CameraDepth;
  ColorRange depth_range;
  bool reverse = false;
  std::filesystem::path metrics_path;
};

class OccupancyDisplay {
 public:
  OccupancyDisplay(MapReader reader,
                   const CameraModel& camera,
                   const std::filesystem::path& screenshot,
                   double max_pose_age_sec = 0.5,
                   const DisplaySettings& settings = {});
  ~OccupancyDisplay();

  bool closed() const;
  void stop();
  std::string error() const;

  void publishCurrentFrame(const cv::Mat& bgr,
                           const ego_pose_bridge::Record& pose,
                           std::uint64_t capture_index,
                           TimePoint received,
                           bool verified);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ego_occupancy
