#include "orbbec_timing.h"

#include <stdexcept>

int main() {
  using namespace orb_live;
  TimingEvent event{7, 1'000, TimingOutcome::SkewRejected,
                    FrameTiming{40, 12'000, 2'000'000},
                    FrameTiming{51, 23'000, 2'018'001}};
  if (formatTimingEvent(event) !=
      "ORBBEC_TIMING,7,1000,skew_rejected,40,12000,2000000,51,23000,2018001,-18001\n") {
    throw std::runtime_error("paired diagnostics must preserve both clocks and the signed offset");
  }
  event.outcome = TimingOutcome::Incomplete;
  event.depth.reset();
  if (formatTimingEvent(event) != "ORBBEC_TIMING,7,1000,incomplete,40,12000,2000000,,,,\n") {
    throw std::runtime_error("incomplete diagnostics must keep the available stream");
  }
}
