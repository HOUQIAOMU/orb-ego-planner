#pragma once

#include <optional>
#include <string>

#include "live_source.h"

namespace ego_occupancy {

struct CaptureMetadata {
  double color_sec;
  double depth_sec;
  std::uint64_t color_number;
  std::uint64_t depth_number;
  int color_domain;
  int depth_domain;
  TimePoint received;
};

class CaptureBackend {
 public:
  virtual ~CaptureBackend() = default;

  virtual void writeDeviceMetadata(cv::FileStorage& metadata) const = 0;
  virtual std::optional<CaptureMetadata> read(int timeout_ms) = 0;
  virtual std::string frameTimeoutDetails() const { return {}; }
  // Called after synchronization and warm-up checks, while the SDK frames are still alive.
  virtual void copyImages(orb_live::RGBDFrame& output) = 0;
  virtual void stop() = 0;
};

std::unique_ptr<CaptureBackend> openRealSense(const Options&, const CameraModel&, int fps);
std::unique_ptr<CaptureBackend> openOrbbec(const Options&, const CameraModel&, int fps);

}  // namespace ego_occupancy
