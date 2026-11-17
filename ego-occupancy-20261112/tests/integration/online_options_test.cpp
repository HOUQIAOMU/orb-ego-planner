#include <ego_occupancy/options.h>

#include <iostream>
#include <stdexcept>
int main() {
  try {
    auto options = ego_occupancy::parseOptions({"--live", "--lock-root", "/tmp", "--vocabulary",
                                                "v", "--camera", "c", "--map-config", "m",
                                                "--output", "o", "--planning-config", "p",
                                                "--probe-start-m", "0.5", "--probe-goal-m", "0.9"});
    if (options.planning_config != "p" || options.probe_start_m != .5 || options.probe_goal_m != .9)
      throw std::runtime_error("planning options missing");
    bool rejected = false;
    try {
      ego_occupancy::parseOptions({"--live", "--lock-root", "/tmp", "--vocabulary", "v", "--camera",
                                   "c", "--map-config", "m", "--output", "o", "--planning-config",
                                   "p", "--probe-start-m", "1", "--probe-goal-m", "0.4"});
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    if (!rejected) throw std::runtime_error("invalid probe accepted");
    std::cout << "PASS online options\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
