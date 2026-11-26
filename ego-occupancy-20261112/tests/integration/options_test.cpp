#include <ego_occupancy/options.h>

#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ego_occupancy;

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      throw std::runtime_error(#condition);                                    \
    }                                                                          \
  } while (false)

Options Parse(std::initializer_list<const char*> arguments) {
  std::vector<std::string> values;
  for (const auto* argument : arguments) {
    values.emplace_back(argument);
  }
  return parseOptions(values);
}

template <class Function>
void ExpectFailure(Function function) {
  bool caught = false;
  try {
    function();
  } catch (const std::exception&) {
    caught = true;
  }
  CHECK(caught);
}

int main() {
  try {
    const auto options = Parse({"--dataset",
                                "/d",
                                "--vocabulary",
                                "/v",
                                "--camera",
                                "/c",
                                "--map-config",
                                "/m",
                                "--output",
                                "/o",
                                "--check",
                                "--no-viewer",
                                "--max-frames",
                                "12"});
    CHECK(!options.live && options.check && !options.viewer &&
          options.max_frames == 12);

    ExpectFailure([] {
      Parse({"--live", "--dataset", "/d", "--vocabulary", "v", "--camera", "c",
             "--map-config", "m", "--output", "o"});
    });
    ExpectFailure([] {
      Parse({"--live", "--vocabulary", "v", "--camera", "c", "--map-config", "m",
             "--output", "o", "--max-seconds", "nan"});
    });
    ExpectFailure([] {
      Parse({"--live", "--vocabulary", "v", "--camera", "c", "--map-config", "m",
             "--output", "o", "--stride", "0"});
    });
    for (const char* mode : {"depth", "axis", "flat"}) {
      const auto colored = Parse({"--dataset", "/d", "--vocabulary", "/v",
                                   "--camera", "/c", "--map-config", "/m",
                                   "--output", "/o", "--display-color", mode});
      CHECK(colored.display_color == mode);
    }
    ExpectFailure([] {
      Parse({"--dataset", "/d", "--vocabulary", "/v", "--camera", "/c",
             "--map-config", "/m", "--output", "/o", "--display-color", "wrong"});
    });
    ExpectFailure([] { Parse({"--bogus"}); });
    CHECK(Parse({"--help"}).help);
    const auto live = Parse({"--live", "--lock-root", "/locks", "--vocabulary", "v",
                             "--camera", "c", "--map-config", "m", "--output", "o"});
    CHECK(live.serial == "042222071908");

    const auto orbbec = Parse({"--live", "--camera-backend", "orbbec", "--lock-root", "/locks",
                               "--vocabulary", "v", "--camera", "c", "--map-config", "m",
                               "--output", "o"});
    CHECK(orbbec.live && orbbec.serial.empty() && !orbbec.record_hz);
    const std::vector<std::string> live_args{
        "--live", "--camera-backend", "orbbec", "--lock-root", "/locks",
        "--vocabulary", "v", "--camera", "c", "--map-config", "m", "--output", "o"};
    auto timingArgs = live_args;
    timingArgs.emplace_back("--orbbec-timing");
    CHECK(parseOptions(timingArgs).orbbec_timing);
    timingArgs[2] = "realsense";
    ExpectFailure([&] { parseOptions(timingArgs); });
    ExpectFailure([] {
      Parse({"--dataset", "/d", "--vocabulary", "v", "--camera", "c",
             "--map-config", "m", "--output", "o", "--orbbec-timing"});
    });
    auto recordArgs = live_args;
    recordArgs.insert(recordArgs.end(), {"--record-hz", "0"});
    CHECK(parseOptions(recordArgs).record_hz == 0);
    recordArgs.back() = "10";
    CHECK(parseOptions(recordArgs).record_hz == 10);
    for (const char* rate : {"-1", "nan", "91"}) {
      recordArgs.back() = rate;
      ExpectFailure([&] { parseOptions(recordArgs); });
    }
    recordArgs.pop_back();
    ExpectFailure([&] { parseOptions(recordArgs); });
    ExpectFailure([] {
      Parse({"--dataset", "/d", "--vocabulary", "v", "--camera", "c",
             "--map-config", "m", "--output", "o", "--record-hz", "10"});
    });
    ExpectFailure([] {
      Parse({"--dataset", "/d", "--camera-backend", "orbbec", "--vocabulary", "v",
             "--camera", "c", "--map-config", "m", "--output", "o"});
    });

    std::cout << "mode, paths, limits and unknown CLI validation passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
