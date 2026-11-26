#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "orb_rgbd_recording.h"
#include <ego_occupancy/depth.h>
#include <ego_occupancy/options.h>

namespace ego_occupancy {

struct LiveStats {
  std::uint64_t captured = 0;
  std::uint64_t recorded = 0;
  std::uint64_t pair_rejected = 0;
  std::uint64_t color_gaps = 0;
  std::uint64_t depth_gaps = 0;
  std::uint64_t slam_dropped = 0;
  std::uint64_t record_eligible = 0;
  std::uint64_t record_skipped_by_rate = 0;
  std::uint64_t record_queue_dropped = 0;
  std::size_t record_queue_high_water = 0;
  bool recording_failed = false;
};

bool liveBackendAvailable(CameraBackend backend);

class LiveSource {
 public:
  LiveSource(const Options& options, const CameraModel& camera, int fps);
  ~LiveSource();

  bool next(orb_live::FramePtr& frame, std::chrono::milliseconds timeout);
  bool finished() const;
  void stop();
  LiveStats stats() const;
  std::string error() const;
  std::string recordingError() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ego_occupancy
