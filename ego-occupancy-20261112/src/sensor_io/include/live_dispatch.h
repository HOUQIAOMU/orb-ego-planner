#pragma once

#include "orb_rgbd_recording.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>

namespace orb_live {

inline bool isFreshForTracking(const RGBDFrame& frame, SteadyClock::time_point now) {
  return now - frame.received <= std::chrono::milliseconds(100);
}

struct DispatchStats {
  std::uint64_t live_dropped;
  std::uint64_t record_eligible;
  std::uint64_t record_skipped_by_rate;
  std::uint64_t record_queue_dropped;
  std::size_t record_queue_high_water;
};

class LiveDispatch {
 public:
  // An unset rate preserves the original full-rate, loss-intolerant recording.
  LiveDispatch(bool newest_only, std::optional<double> record_hz)
      : record_hz_(record_hz),
        live_(newest_only ? 1 : 2),
        disk_(record_hz ? 8 : 64, true),
        recording_(record_hz.value_or(1.0) > 0) {}

  bool publish(FramePtr frame) {
    if (!record_hz_) {
      ++record_eligible_;
      return disk_.push(frame) && live_.push(std::move(frame));
    }

    if (!live_.push(frame)) return false;
    if (recording_ && selectForRecording(frame->color_sec)) {
      ++record_eligible_;
      disk_.push(std::move(frame));
    }
    return true;
  }

  bool next(FramePtr& frame, std::chrono::milliseconds timeout) {
    return live_.pop_for(frame, timeout);
  }

  bool nextRecording(FramePtr& frame, std::chrono::milliseconds timeout) {
    return disk_.pop_for(frame, timeout);
  }

  bool finished() const { return live_.finished(); }
  bool recordingFinished() const { return disk_.finished(); }
  bool recordingEnabled() const { return recording_; }
  bool bestEffortRecording() const { return record_hz_.has_value(); }

  void close() {
    live_.close();
    disk_.close();
  }

  void stopRecording() {
    recording_ = false;
    disk_.close(true);
  }

  DispatchStats stats() const {
    return {live_.dropped(), record_eligible_, record_skipped_by_rate_,
            disk_.dropped(), disk_.high_water()};
  }

 private:
  bool selectForRecording(double time) {
    if (!origin_) origin_ = time;
    // Pair timestamps are doubles. The tolerance keeps exact sampling boundaries
    // from landing in the preceding slot after subtracting the origin.
    const double slot = std::floor((time - *origin_) * *record_hz_ + 1e-9);
    if (slot <= last_slot_) {
      ++record_skipped_by_rate_;
      return false;
    }
    last_slot_ = slot;
    return true;
  }

  std::optional<double> record_hz_;
  BoundedQueue<FramePtr> live_;
  BoundedQueue<FramePtr> disk_;
  std::atomic<bool> recording_;
  std::optional<double> origin_;
  double last_slot_ = -1;
  std::atomic<std::uint64_t> record_eligible_{0};
  std::atomic<std::uint64_t> record_skipped_by_rate_{0};
};

}  // namespace orb_live
