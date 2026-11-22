#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "pose_bridge.h"
#include <ego_occupancy/depth.h>

namespace ego_occupancy {

struct WorkerStats {
  std::uint64_t submitted = 0;
  std::uint64_t fused = 0;
  std::uint64_t rejected = 0;
  std::uint64_t dropped = 0;
  std::uint64_t publish_skipped = 0;
  std::uint64_t source_events = 0;
  double last_fuse_ms = 0.0;
  double last_inflate_ms = 0.0;
  double last_publish_ms = 0.0;
  double last_age_ms = 0.0;
  std::string error;
};

class MappingWorker {
 public:
  MappingWorker(const Config& config,
                const CameraModel& camera,
                const std::filesystem::path& diagnostics = {});
  ~MappingWorker();

  MappingWorker(const MappingWorker&) = delete;
  MappingWorker& operator=(const MappingWorker&) = delete;

  bool submit(const cv::Mat& owned_depth,
              const ego_pose_bridge::Record& pose,
              std::uint64_t capture_index,
              TimePoint received);

  bool waitFor(std::uint64_t capture_index, std::chrono::milliseconds timeout);

  void trackingLost(const std::string& reason);
  void stop();

  MapReader reader() const;
  WorkerStats stats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ego_occupancy
