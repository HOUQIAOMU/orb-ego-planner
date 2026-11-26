#pragma once

#include "orb_live_common.h"

#include <cstdio>
#include <sstream>
#include <thread>

namespace orb_live {

enum class TimingOutcome { Accepted, SkewRejected, NoncurrentOrInvalid, Incomplete, SdkTimeout };

struct FrameTiming {
  std::uint64_t index;
  std::uint64_t device_us;
  std::uint64_t host_us;
};

struct TimingEvent {
  std::uint64_t sequence = 0;
  std::int64_t dequeue_monotonic_ns = 0;
  TimingOutcome outcome = TimingOutcome::SdkTimeout;
  std::optional<FrameTiming> color;
  std::optional<FrameTiming> depth;
};

inline const char* timingOutcomeName(TimingOutcome value) {
  switch (value) {
    case TimingOutcome::Accepted: return "accepted";
    case TimingOutcome::SkewRejected: return "skew_rejected";
    case TimingOutcome::NoncurrentOrInvalid: return "noncurrent_or_invalid";
    case TimingOutcome::Incomplete: return "incomplete";
    case TimingOutcome::SdkTimeout: return "sdk_timeout";
  }
  return "unknown";
}

inline std::string formatTimingEvent(const TimingEvent& event) {
  std::ostringstream line;
  line << "ORBBEC_TIMING," << event.sequence << ',' << event.dequeue_monotonic_ns
       << ',' << timingOutcomeName(event.outcome);
  for (const auto* frame : {&event.color, &event.depth}) {
    if (*frame) {
      line << ',' << (*frame)->index << ',' << (*frame)->device_us << ',' << (*frame)->host_us;
    } else {
      line << ",,,";
    }
  }
  line << ',';
  if (event.color && event.depth && event.color->host_us && event.depth->host_us) {
    const auto color = event.color->host_us;
    const auto depth = event.depth->host_us;
    if (color < depth) line << '-';
    line << (color >= depth ? color - depth : depth - color);
  }
  line << '\n';
  return line.str();
}

class OrbbecTimingLog {
 public:
  OrbbecTimingLog() {
    std::fputs("ORBBEC_TIMING_COLUMNS,sequence,dequeue_monotonic_ns,outcome,"
               "color_index,color_device_us,color_host_us,depth_index,depth_device_us,depth_host_us,"
               "color_minus_depth_host_us\n", stderr);
    consumer_ = std::thread([this] {
      TimingEvent event;
      while (!events_.finished()) {
        if (events_.pop_for(event, std::chrono::milliseconds(100))) {
          const auto line = formatTimingEvent(event);
          std::fputs(line.c_str(), stderr);
        }
      }
    });
  }

  ~OrbbecTimingLog() { finish(); }

  void push(TimingEvent event) {
    event.sequence = ++sequence_;
    events_.push(std::move(event));
  }

  void finish() {
    if (!consumer_.joinable()) return;
    events_.close();
    consumer_.join();
    std::fprintf(stderr, "ORBBEC_TIMING_END candidates=%llu dropped=%llu\n",
                 static_cast<unsigned long long>(sequence_),
                 static_cast<unsigned long long>(events_.dropped()));
  }

 private:
  // Copy only timestamps and IDs. Terminal/log IO stays off the capture thread.
  BoundedQueue<TimingEvent> events_{512, true};
  std::thread consumer_;
  std::uint64_t sequence_ = 0;
};

}  // namespace orb_live
