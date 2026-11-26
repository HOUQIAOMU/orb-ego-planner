#include <ego_occupancy/io.h>
#include <ego_occupancy/options.h>
#include <ego_occupancy/worker.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/resource.h>
#include <unistd.h>

#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef EGO_HAS_PLANNER
#include "online_planning_session.h"
#include "planning_io.h"
#endif

#ifdef EGO_HAS_DISPLAY
#include <ego_occupancy/display.h>
#endif

#include "System.h"
#include "live_dispatch.h"
#include "live_source.h"

namespace fs = std::filesystem;
using namespace ego_occupancy;

namespace {

volatile std::sig_atomic_t g_interrupted = 0;

void SignalHandler(int) { g_interrupted = 1; }

struct Input {
  double color_sec = 0.0;
  double depth_sec = 0.0;
  fs::path color;
  fs::path depth;
};

fs::path ContainedPath(const fs::path& root, const std::string& name) {
  fs::path path(name);
  if (path.is_absolute()) {
    throw std::invalid_argument("absolute sequence image path rejected");
  }

  path = fs::canonical(root / path);
  const auto relative = path.lexically_relative(root);
  if (relative.empty() || *relative.begin() == ".." || !fs::is_regular_file(path)) {
    throw std::invalid_argument("sequence path escapes data directory");
  }
  return path;
}

std::vector<Input> LoadSequence(const Options& options, const CameraModel& camera) {
  const auto root = fs::canonical(options.dataset);
  std::ifstream file(root / "associations.txt");
  if (!file) {
    throw std::invalid_argument("missing associations.txt");
  }

  std::vector<Input> inputs;
  std::string line;
  double previous_color = -1.0;
  double previous_depth = -1.0;
  while (std::getline(file, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    if (line.find_first_not_of(" \t\r") == std::string::npos) {
      continue;
    }

    Input input;
    std::string color_name;
    std::string depth_name;
    std::string extra;
    std::istringstream stream(line);
    if (!(stream >> input.color_sec >> color_name >> input.depth_sec >> depth_name) ||
        (stream >> extra) || !std::isfinite(input.color_sec) || !std::isfinite(input.depth_sec) ||
        input.color_sec <= previous_color || input.depth_sec <= previous_depth ||
        std::abs(input.color_sec - input.depth_sec) > 0.010) {
      throw std::invalid_argument("invalid/unpaired/nonmonotonic RGB-D association");
    }

    input.color = ContainedPath(root, color_name);
    input.depth = ContainedPath(root, depth_name);
    inputs.push_back(input);
    previous_color = input.color_sec;
    previous_depth = input.depth_sec;
  }

  if (inputs.size() < 2) {
    throw std::invalid_argument("at least two RGB-D pairs required");
  }

  const auto color = cv::imread(inputs[0].color.string(), cv::IMREAD_UNCHANGED);
  const auto depth = cv::imread(inputs[0].depth.string(), cv::IMREAD_UNCHANGED);
  if (color.type() != CV_8UC3 || depth.type() != CV_16UC1 ||
      color.size() != cv::Size(camera.width, camera.height) || depth.size() != color.size()) {
    throw std::invalid_argument("sequence images differ from BGR8/aligned Z16 calibration");
  }
  return inputs;
}

class CameraLock {
 public:
  explicit CameraLock(const fs::path& root, CameraBackend backend) {
    try {
      const char* camera_lock =
          backend == CameraBackend::Orbbec ? ".orbbec-capture.lock" : ".d435i-capture.lock";
      for (const char* name : {camera_lock, ".orb-realtime.lock"}) {
        const auto path = root / name;
        const int descriptor = open(path.c_str(), O_CREAT | O_RDWR, 0600);
        if (descriptor < 0) {
          throw std::runtime_error("cannot open camera lock: " + path.string());
        }
        descriptors_.push_back(descriptor);
        if (flock(descriptor, LOCK_EX | LOCK_NB) != 0) {
          throw std::runtime_error("camera or SLAM already in use: " + path.string());
        }
      }
    } catch (...) {
      for (const int descriptor : descriptors_) {
        close(descriptor);
      }
      throw;
    }
  }

  ~CameraLock() {
    for (const int descriptor : descriptors_) {
      close(descriptor);
    }
  }

  CameraLock(const CameraLock&) = delete;
  CameraLock& operator=(const CameraLock&) = delete;

 private:
  std::vector<int> descriptors_;
};

int Run(int argc, char** argv) {
  std::vector<std::string> arguments;
  for (int i = 1; i < argc; ++i) {
    arguments.emplace_back(argv[i]);
  }

  Options options;
  CameraModel camera;
  Config config;
  std::vector<Input> inputs;
  int fps = 0;

  try {
    options = parseOptions(arguments);
    if (options.help) {
      std::cout << usage() << '\n';
      return 0;
    }

    if (!fs::is_regular_file(options.vocabulary)) {
      throw std::invalid_argument("vocabulary file missing");
    }
    if (fs::exists(options.output)) {
      throw std::invalid_argument("output must be new; refusing overwrite");
    }

    camera = readCameraModel(options.camera.string(), options.stride);
    config = readMapConfig(options.map_config.string());
    // Validate dimensions/budget without allocating a full mapper during --check.
    validateConfig(config);

    if (!options.live) {
      inputs = LoadSequence(options, camera);
    } else {
      if (!liveBackendAvailable(options.camera_backend))
        throw std::invalid_argument("selected live camera backend was not built");
      cv::FileStorage storage(options.camera.string(), cv::FileStorage::READ);
      const double rate = static_cast<double>(storage["Camera.fps"]);
      if (!std::isfinite(rate) || rate < 1 || rate > 90 || std::floor(rate) != rate) {
        throw std::invalid_argument("invalid Camera.fps");
      }
      fps = static_cast<int>(rate);
      if (!fs::is_directory(options.lock_root)) {
        throw std::invalid_argument("live lock root missing");
      }
    }

#ifndef EGO_HAS_DISPLAY
    if (options.viewer && !options.check) {
      throw std::invalid_argument(
          "build has no display; use --no-viewer or build EGO_BUILD_DISPLAY=ON");
    }
#endif

#ifdef EGO_HAS_PLANNER
    if (!options.planning_config.empty()) {
      const auto pc = ego_planner::app::loadPlanningConfig(options.planning_config);
      ego_planner::validateConfig(pc);
    }
#else
    if (!options.planning_config.empty())
      throw std::invalid_argument("build has no planner; use the online build entry");
#endif
    std::cout << "INPUT_VALIDATED mode=" << (options.live ? "live" : "replay")
              << " pairs=" << inputs.size() << " resolution_m=" << config.resolution
              << " stride=" << options.stride << " no_ROS no_IMU\n";
    if (options.check) {
      return 0;
    }
  } catch (const std::exception& error) {
    std::cerr << "Input rejected: " << error.what() << '\n';
    return 2;
  }

  std::unique_ptr<CameraLock> camera_lock;
  if (options.live) {
    camera_lock = std::make_unique<CameraLock>(options.lock_root, options.camera_backend);
  }

  fs::create_directories(options.output);
  fs::copy_file(options.camera, options.output / "camera.yaml");
  fs::copy_file(options.map_config, options.output / "map-config.yaml");

  const std::uint64_t session =
      static_cast<std::uint64_t>(Clock::now().time_since_epoch().count()) ^
      static_cast<std::uint64_t>(getpid());
  ego_pose_bridge::Initialize(session ? session : 1);

  MappingWorker mapping(config, camera, options.output / "mapping.csv");
#ifdef EGO_HAS_PLANNER
  std::unique_ptr<ego_planner::app::OnlinePlanningSession> planning;
  if (!options.planning_config.empty()) {
    fs::copy_file(options.planning_config, options.output / "planning-config.yaml");
    planning = std::make_unique<ego_planner::app::OnlinePlanningSession>(
        mapping.reader(), ego_planner::app::loadPlanningConfig(options.planning_config),
        options.output / "planning", options.probe_start_m, options.probe_goal_m);
  }
#endif

#ifdef EGO_HAS_DISPLAY
  std::unique_ptr<OccupancyDisplay> display;
  if (options.viewer) {
    DisplaySettings display_settings;
    if (options.display_color == "axis") {
      display_settings.color_mode = ColorMode::MapAxis;
    } else if (options.display_color == "flat") {
      display_settings.color_mode = ColorMode::Flat;
    }
#ifdef EGO_HAS_PLANNER
    if (planning) {
      display_settings.planning_layer = [&planning] { return planning->layer(); };
      display_settings.trigger_planning = [&planning] { planning->trigger(); };
      display_settings.clear_planning = [&planning] { planning->clear(); };
    }
#endif
    display_settings.metrics_path = options.output / "display.csv";
    display = std::make_unique<OccupancyDisplay>(mapping.reader(), camera,
                                                 options.output / "occupancy.png",
                                                 config.max_age_sec, display_settings);
  }
#endif

  // Preserve original core and GUI baseline. The installed ORB Shutdown does not
  // join all workers; retain this object until process-level _Exit after our IO
  // joins.
  auto* slam = new ORB_SLAM3::System(options.vocabulary.string(), options.camera.string(),
                                     ORB_SLAM3::System::RGBD, false);

  std::unique_ptr<LiveSource> live;
  if (options.live) {
    live = std::make_unique<LiveSource>(options, camera, fps);
  }

  std::ofstream tracking(options.output / "poses.csv");
  tracking.exceptions(std::ios::failbit | std::ios::badbit);
  tracking << "capture_index,orb_frame,timestamp_sec,depth_timestamp_sec,state,captured,"
              "pose_frame_verified,session,map_token,revision,track_ms,tx_m,ty_m,tz_m,slam_input_host_age_ms\n"
           << std::setprecision(std::numeric_limits<double>::max_digits10);

  std::uint64_t processed = 0;
  std::uint64_t valid = 0;
  std::uint64_t slam_stale_dropped = 0;
  int tracking_state = -1;
  std::uint64_t tracking_revision = 0;
  std::uint64_t last_submitted = 0;
  bool have_submitted = false;
  double last_map_time = -std::numeric_limits<double>::infinity();
  const auto started = Clock::now();
  auto last_progress = started;
  std::string failure;

  const auto checkWorkersAndReport = [&](Clock::time_point now) {
#ifdef EGO_HAS_PLANNER
    if (planning) {
      const auto error = planning->error();
      if (!error.empty()) throw std::runtime_error("online planning failed: " + error);
    }
#endif
    const auto stats = mapping.stats();
    if (!stats.error.empty()) throw std::runtime_error(stats.error);
    if (now - last_progress < std::chrono::seconds(1)) return;
    const auto input = live ? live->stats() : LiveStats{};
    std::cout << "MAP_PROGRESS processed=" << processed << " verified=" << valid
              << " state=" << tracking_state << " revision=" << tracking_revision
              << " fused=" << stats.fused << " rejected=" << stats.rejected
              << " queue_drop=" << stats.dropped << " fuse_ms=" << stats.last_fuse_ms
              << " inflate_ms=" << stats.last_inflate_ms << " age_ms=" << stats.last_age_ms
              << " captured=" << input.captured << " recorded=" << input.recorded
              << " record_queue_dropped=" << input.record_queue_dropped
              << " slam_stale_dropped=" << slam_stale_dropped << '\n';
    last_progress = now;
  };

  std::cout << "MAPPING_READY mode=" << (options.live ? "live" : "replay")
            << " output=" << fs::absolute(options.output)
            << "; Ctrl-C stops input and drains recording\n";

  try {
    while (!g_interrupted) {
#ifdef EGO_HAS_DISPLAY
      if (display && display->closed()) {
        break;
      }
#endif

      if (options.max_seconds > 0 &&
          std::chrono::duration<double>(Clock::now() - started).count() >= options.max_seconds) {
        break;
      }

      orb_live::FramePtr frame;
      if (live) {
        if (!live->next(frame, std::chrono::milliseconds(100))) {
          if (live->finished()) break;
          checkWorkersAndReport(Clock::now());
          continue;
        }
      } else {
        if (processed >= inputs.size() ||
            (options.max_frames && processed >= static_cast<std::uint64_t>(options.max_frames))) {
          break;
        }

        const auto& input = inputs[processed];
        auto replay_frame = std::make_shared<orb_live::RGBDFrame>();
        replay_frame->received = Clock::now();
        replay_frame->index = processed;
        replay_frame->color_sec = input.color_sec;
        replay_frame->depth_sec = input.depth_sec;
        replay_frame->color = cv::imread(input.color.string(), cv::IMREAD_UNCHANGED);
        replay_frame->depth = cv::imread(input.depth.string(), cv::IMREAD_UNCHANGED);
        frame = replay_frame;
      }

      if (frame->color.type() != CV_8UC3 || frame->depth.type() != CV_16UC1 ||
          frame->color.size() != cv::Size(camera.width, camera.height) ||
          frame->depth.size() != frame->color.size()) {
        throw std::runtime_error("RGB-D encoding or size changed");
      }

      const auto before = Clock::now();
      if (live && options.camera_backend == CameraBackend::Orbbec &&
          !orb_live::isFreshForTracking(*frame, before)) {
        ++slam_stale_dropped;
        checkWorkersAndReport(before);
        continue;
      }
      slam->TrackRGBD(frame->color, frame->depth, frame->color_sec);
      const auto after = Clock::now();
      const auto record = ego_pose_bridge::Last();
      ++processed;
      tracking_state = record.tracking_state;
      tracking_revision = record.revision;

      const bool bound = record.captured && record.timestamp_sec == frame->color_sec &&
                         ego_pose_bridge::IsCurrent(record);
#ifdef EGO_HAS_PLANNER
      if (planning) planning->update(record, frame->index, frame->received, bound);
#endif
      if (bound) {
        ++valid;
      } else {
        mapping.trackingLost("tracking not valid/current");
      }

#ifdef EGO_HAS_DISPLAY
      if (display) {
        display->publishCurrentFrame(frame->color, record, frame->index, frame->received, bound);
      }
#endif

      const Eigen::Map<const Eigen::Matrix4f> transform(record.T_world_camera.data());
      tracking << frame->index << ',' << record.frame_id << ',' << frame->color_sec << ','
               << frame->depth_sec << ',' << record.tracking_state << ',' << record.captured << ','
               << bound << ',' << record.session << ',' << record.map_token << ','
               << record.revision << ','
               << std::chrono::duration<double, std::milli>(after - before).count() << ','
               << transform(0, 3) << ',' << transform(1, 3) << ',' << transform(2, 3) << ',';
      if (live) {
        tracking << std::chrono::duration<double, std::milli>(before - frame->received).count();
      }
      tracking << '\n';
      tracking.flush();

      if (bound && (frame->color_sec - last_map_time >= 1.0 / options.mapping_hz)) {
        if (mapping.submit(frame->depth, record, frame->index, frame->received)) {
          last_submitted = frame->index;
          have_submitted = true;
          last_map_time = frame->color_sec;
          if (!options.live && !mapping.waitFor(last_submitted, std::chrono::seconds(10))) {
            throw std::runtime_error("offline mapping worker timeout");
          }
        }
      }

      checkWorkersAndReport(after);
    }
  } catch (const std::exception& error) {
    failure = error.what();
  }

  // Flush and final map export are outside the timed input window.
  const auto input_ended = Clock::now();
  if (live) {
    live->stop();
    if (!live->error().empty()) {
      failure = live->error();
    }
  }
  if (have_submitted && !mapping.waitFor(last_submitted, std::chrono::seconds(10)) &&
      failure.empty()) {
    failure = "mapping drain timeout";
  }

#ifdef EGO_HAS_DISPLAY
  if (display) display->stop();
#endif
#ifdef EGO_HAS_PLANNER
  if (planning) {
    planning->stop();
    if (!planning->error().empty()) failure = "online planning failed: " + planning->error();
  }
#endif
  mapping.stop();
  auto stats = mapping.stats();
  if (!stats.error.empty()) {
    failure = stats.error;
  }

#ifdef EGO_HAS_DISPLAY
  if (display) {
    display->stop();
    if (!display->error().empty()) {
      failure = display->error();
    }
  }
#endif

  const auto snapshot = mapping.reader().latestForDisplay();
  if (snapshot) {
    saveSnapshot(*snapshot, options.output / "map.egovox");
    exportPly(*snapshot, options.output / "occupied.ply");
    exportPly(*snapshot, options.output / "inflated.ply", true);
    const auto loaded = loadSnapshot(options.output / "map.egovox");
    if (loaded->voxelCount() != snapshot->voxelCount()) {
      throw std::runtime_error("snapshot reload failed");
    }
  }

  struct rusage memory{};
  getrusage(RUSAGE_SELF, &memory);
  LiveStats live_stats;
  if (live) {
    live_stats = live->stats();
  }

  cv::FileStorage summary((options.output / "summary.yml").string(), cv::FileStorage::WRITE);
  if (!summary.isOpened()) {
    throw std::runtime_error("cannot write summary");
  }
  summary << "mode" << (options.live ? "live" : "replay") << "processed"
          << static_cast<double>(processed) << "verified_poses" << static_cast<double>(valid)
          << "fused" << static_cast<double>(stats.fused) << "mapping_rejected"
          << static_cast<double>(stats.rejected) << "mapping_queue_dropped"
          << static_cast<double>(stats.dropped) << "snapshot_publish_skipped"
          << static_cast<double>(stats.publish_skipped) << "source_events"
          << static_cast<double>(stats.source_events) << "captured"
          << static_cast<double>(live_stats.captured) << "recorded"
          << static_cast<double>(live_stats.recorded) << "pair_rejected"
          << static_cast<double>(live_stats.pair_rejected) << "sdk_color_gaps"
          << static_cast<double>(live_stats.color_gaps) << "sdk_depth_gaps"
          << static_cast<double>(live_stats.depth_gaps) << "slam_queue_dropped"
          << static_cast<double>(live_stats.slam_dropped)
          << "slam_stale_dropped" << static_cast<double>(slam_stale_dropped)
          << "record_eligible" << static_cast<double>(live_stats.record_eligible)
          << "record_skipped_by_rate" << static_cast<double>(live_stats.record_skipped_by_rate)
          << "record_queue_dropped" << static_cast<double>(live_stats.record_queue_dropped)
          << "record_queue_high_water" << static_cast<double>(live_stats.record_queue_high_water)
          << "recording_failure" << (live ? live->recordingError() : std::string{})
          << "peak_rss_mib"
          << memory.ru_maxrss / 1024.0 << "has_snapshot" << static_cast<int>(bool(snapshot))
          << "final_snapshot_is_live" << 0 << "failure" << failure;
  if (live) {
    // FileStorage stores the integer stamps as text to preserve nanosecond precision.
    const auto monotonicNs = [](Clock::time_point stamp) {
      return std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                stamp.time_since_epoch()).count());
    };
    summary << "window_start_monotonic_ns" << monotonicNs(started)
            << "window_end_monotonic_ns" << monotonicNs(input_ended)
            << "mapping_target_hz" << options.mapping_hz;
  }

  std::cout << "MAPPING_FINISHED processed=" << processed << " verified=" << valid
            << " fused=" << stats.fused << " snapshot=" << bool(snapshot)
            << " captured=" << live_stats.captured << " recorded=" << live_stats.recorded
            << " record_queue_dropped=" << live_stats.record_queue_dropped
            << " slam_stale_dropped=" << slam_stale_dropped
            << " peak_rss_mib=" << memory.ru_maxrss / 1024.0 << " failure=" << failure << '\n';

  if (!failure.empty()) {
    return 1;
  }
  return snapshot && stats.fused ? 0 : 3;
}

}  // namespace

int main(int argc, char** argv) {
  std::cout << std::unitbuf;
  std::cerr << std::unitbuf;
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  int status = 1;
  try {
    status = Run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "Mapping failed: " << error.what() << '\n';
  }

  std::cout.flush();
  std::cerr.flush();
  std::_Exit(status);
}
