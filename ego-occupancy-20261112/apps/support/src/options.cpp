#include <ego_occupancy/options.h>

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace ego_occupancy {

std::string usage() {
  return "ego-rgbd-map --vocabulary VOC --camera CAMERA.yaml --map-config MAP.yaml "
         "--output NEW_DIR (--dataset DATA_DIR | --live --lock-root DIR) [--check] "
         "[--no-viewer] [--max-frames N] [--max-seconds S] [--stride N] "
         "[--planning-config YAML --probe-start-m M --probe-goal-m M] [--mapping-hz HZ] [--warmup "
         "N] [--record-hz HZ (live; 0 disables recording)] [--camera-backend realsense|orbbec] [--orbbec-timing] [--serial SERIAL] [--display-color depth|axis|flat]";
}

Options parseOptions(const std::vector<std::string>& arguments) {
  Options options;
  bool backend_selected = false;

  const auto realValue = [](const std::string& text) {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value)) {
      throw std::invalid_argument("expected finite number");
    }
    return value;
  };

  const auto integerValue = [&realValue](const std::string& text) {
    const double value = realValue(text);
    if (value < 0 || value > 10'000'000 || std::floor(value) != value) {
      throw std::invalid_argument("invalid integer");
    }
    return static_cast<int>(value);
  };

  for (std::size_t i = 0; i < arguments.size(); ++i) {
    const auto& argument = arguments[i];
    if (argument == "--help" || argument == "-h") {
      options.help = true;
      return options;
    }

    if (argument == "--live") {
      options.live = true;
    } else if (argument == "--check") {
      options.check = true;
    } else if (argument == "--orbbec-timing") {
      options.orbbec_timing = true;
    } else if (argument == "--no-viewer") {
      options.viewer = false;
    } else {
      if (++i == arguments.size()) {
        throw std::invalid_argument("missing value for " + argument);
      }
      const auto& value = arguments[i];
      if (argument == "--planning-config") {
        options.planning_config = value;
      } else if (argument == "--probe-start-m") {
        options.probe_start_m = realValue(value);
      } else if (argument == "--probe-goal-m") {
        options.probe_goal_m = realValue(value);
      } else if (argument == "--vocabulary") {
        options.vocabulary = value;
      } else if (argument == "--camera") {
        options.camera = value;
      } else if (argument == "--map-config") {
        options.map_config = value;
      } else if (argument == "--output") {
        options.output = value;
      } else if (argument == "--dataset") {
        options.dataset = value;
      } else if (argument == "--lock-root") {
        options.lock_root = value;
      } else if (argument == "--max-frames") {
        options.max_frames = integerValue(value);
      } else if (argument == "--stride") {
        options.stride = integerValue(value);
      } else if (argument == "--warmup") {
        options.warmup = integerValue(value);
      } else if (argument == "--max-seconds") {
        options.max_seconds = realValue(value);
      } else if (argument == "--mapping-hz") {
        options.mapping_hz = realValue(value);
      } else if (argument == "--display-color") {
        options.display_color = value;
      } else if (argument == "--record-hz") {
        options.record_hz = realValue(value);
      } else if (argument == "--camera-backend") {
        backend_selected = true;
        if (value == "realsense")
          options.camera_backend = CameraBackend::RealSense;
        else if (value == "orbbec")
          options.camera_backend = CameraBackend::Orbbec;
        else
          throw std::invalid_argument("unknown camera backend: " + value);
      } else if (argument == "--serial") {
        if (value.empty()) throw std::invalid_argument("empty camera serial");
        options.serial = value;
      } else {
        throw std::invalid_argument("unknown option: " + argument);
      }
    }
  }

  if (options.live == !options.dataset.empty() || options.vocabulary.empty() ||
      options.camera.empty() || options.map_config.empty() || options.output.empty() ||
      options.probe_start_m <= 0 || options.probe_goal_m <= options.probe_start_m ||
      options.probe_goal_m > 4.5 || options.stride < 1 || options.stride > 32 ||
      options.max_seconds < 0 ||
      (options.display_color != "depth" && options.display_color != "axis" &&
       options.display_color != "flat") ||
      options.mapping_hz <= 0 || options.mapping_hz > 120 || (!options.live && backend_selected) ||
      (options.record_hz && (!options.live || *options.record_hz < 0 || *options.record_hz > 90)) ||
      (options.live && options.lock_root.empty())) {
    throw std::invalid_argument("invalid mode, paths or limits; " + usage());
  }

  if (options.orbbec_timing && (!options.live || options.camera_backend != CameraBackend::Orbbec)) {
    throw std::invalid_argument("--orbbec-timing requires live Orbbec capture");
  }

  if (options.camera_backend == CameraBackend::RealSense && options.serial.empty())
    options.serial = "042222071908";
  return options;
}

}  // namespace ego_occupancy
