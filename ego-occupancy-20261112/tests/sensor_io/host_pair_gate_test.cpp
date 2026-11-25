#include "orb_live_common.h"

#include <iostream>
#include <stdexcept>

int main() {
  try {
    orb_live::HostPairGate gate;
    if (!gate.accept(10, 1'000'000, 20, 1'005'000))
      throw std::runtime_error("initial pair inside tolerance must pass");
    if (gate.accept(11, 1'016'000, 20, 1'005'000))
      throw std::runtime_error("a newer color must not reuse the last paired depth");
    if (gate.accept(11, 1'016'000, 21, 1'027'000))
      throw std::runtime_error("receive skew above 10 ms must fail");
    if (!gate.accept(12, 1'031'000, 21, 1'027'000))
      throw std::runtime_error("rejecting one pair must leave its new depth available");
    if (gate.accept(13, 0, 22, 1'044'000) || gate.accept(13, 1'031'000, 22, 1'035'000))
      throw std::runtime_error("absent and nonincreasing host timestamps must fail");
    if (!gate.accept(13, 1'050'000, 22, 1'040'000))
      throw std::runtime_error("accept both arrival orders at the 10 ms limit");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
