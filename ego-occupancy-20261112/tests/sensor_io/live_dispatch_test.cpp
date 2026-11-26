#include "live_dispatch.h"
#include "test_support.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using namespace orb_live;

static FramePtr Frame(std::uint64_t index, double time = 0) {
  auto frame = std::make_shared<RGBDFrame>();
  frame->index = index;
  frame->color_sec = time;
  return frame;
}

int main() {
  LiveDispatch newest(true, 0);
  CHECK(newest.publish(Frame(1)));
  CHECK(newest.publish(Frame(2)));
  CHECK(newest.publish(Frame(3)));
  FramePtr frame;
  CHECK(newest.next(frame, 0ms) && frame->index == 3);
  CHECK(newest.stats().live_dropped == 2);
  CHECK(newest.publish(Frame(4)) && frame->index == 3);
  CHECK(!newest.nextRecording(frame, 0ms));

  RGBDFrame timed;
  timed.received = SteadyClock::time_point{} + 1s;
  CHECK(isFreshForTracking(timed, timed.received + 100ms));
  CHECK(!isFreshForTracking(timed, timed.received + 101ms));

  LiveDispatch sampled(true, 10);
  for (std::uint64_t i = 0; i < 60; ++i) {
    CHECK(sampled.publish(Frame(i, 10.0 + i / 60.0)));
    if (i % 6 == 0) {
      CHECK(sampled.nextRecording(frame, 0ms) && frame->index == i);
    } else {
      CHECK(!sampled.nextRecording(frame, 0ms));
    }
  }
  CHECK(sampled.stats().record_eligible == 10);
  CHECK(sampled.stats().record_skipped_by_rate == 50);
  CHECK(sampled.publish(Frame(60, 30.0)));
  CHECK(sampled.nextRecording(frame, 0ms) && frame->index == 60);
  CHECK(!sampled.nextRecording(frame, 0ms));

  LiveDispatch slow_writer(true, 60);
  for (std::uint64_t i = 0; i < 12; ++i) {
    CHECK(slow_writer.publish(Frame(i, i / 60.0)));
  }
  CHECK(slow_writer.next(frame, 0ms) && frame->index == 11);
  const auto stats = slow_writer.stats();
  CHECK(stats.record_eligible == 12);
  CHECK(stats.record_queue_dropped == 4);
  CHECK(stats.record_queue_high_water == 8);
  CHECK(slow_writer.nextRecording(frame, 0ms) && frame->index == 0);
  slow_writer.stopRecording();
  CHECK(slow_writer.recordingFinished());
  CHECK(slow_writer.publish(Frame(12, 0.2)));
  CHECK(slow_writer.next(frame, 0ms) && frame->index == 12);
  CHECK(slow_writer.stats().record_queue_dropped == 4);

  LiveDispatch legacy(false, std::nullopt);
  CHECK(legacy.publish(Frame(0)) && legacy.publish(Frame(1)) && legacy.publish(Frame(2)));
  CHECK(legacy.next(frame, 0ms) && frame->index == 1);
  for (std::uint64_t i = 3; i < 64; ++i) {
    CHECK(legacy.publish(Frame(i)));
  }
  CHECK(!legacy.publish(Frame(64)));
  CHECK(legacy.stats().record_queue_dropped == 1);
  CHECK(legacy.stats().record_queue_high_water == 64);
}
