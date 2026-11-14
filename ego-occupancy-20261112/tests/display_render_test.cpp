#include <ego_occupancy/display.h>

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/imgcodecs.hpp>

using namespace ego_occupancy;

int main(int argc, char** argv) {
  try {
    // A DISPLAY variable can point at an inaccessible desktop in CI/sandboxes.
    // GUI contracts are opt-in and are exercised on the real RK desktop.
    if (!std::getenv("EGO_TEST_DESKTOP") || !std::getenv("DISPLAY")) {
      return 77;
    }
    if (argc != 2) {
      throw std::invalid_argument("mode required");
    }

    const bool cleanup = std::string(argv[1]) == "cleanup";
    const auto root = std::filesystem::temp_directory_path() /
                      ("ego-display-render-" + std::to_string(getpid()));
    std::filesystem::create_directory(root);
    const auto image_path =
        cleanup ? root / "missing-parent" / "frame.png" : root / "frame.png";

    ego_pose_bridge::Initialize(7799);
    static int map_identity = 0;
    float transform[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    ego_pose_bridge::BeginFrame(1.0);
    ego_pose_bridge::Capture(1, 1.0, &map_identity, transform, 2, true);
    auto pose = ego_pose_bridge::Last();

    Config config;
    config.size = Vec3(2, 2, 4);
    config.resolution = 0.1;
    config.max_age_sec = 10;

    CameraModel camera;
    camera.width = 640;
    camera.height = 480;
    camera.fx = camera.fy = 500;
    camera.cx = 320;
    camera.cy = 240;

    OccupancyMapper mapper(config);
    mapper.reset({pose.session, pose.map_token, pose.revision}, Vec3::Zero());
    OccupancyDisplay display(mapper.reader(), camera, image_path, 10);
    display.publishCurrentFrame(cv::Mat(480, 640, CV_8UC3, cv::Scalar(50, 100, 200)),
                                pose, 1, Clock::now(), true);

    for (int i = 0; i < 3; ++i) {
      RayObservation observation;
      observation.source = {pose.session, pose.map_token, pose.revision};
      observation.timestamp_sec = 1.0 + i * 0.1;
      observation.frame_index = i;
      observation.received = Clock::now();
      observation.rays = {{Vec3(0, 0, 1), true}};
      mapper.integrate(observation);
    }

    const auto deadline = Clock::now() + std::chrono::seconds(10);
    while (Clock::now() < deadline && !display.closed() &&
           !std::filesystem::exists(image_path)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (cleanup) {
      if (!display.closed() ||
          display.error().find("cannot save occupancy screenshot") == std::string::npos) {
        throw std::runtime_error("expected screenshot write failure after window creation");
      }

      // The process remains alive: an abandoned Pangolin context would retain
      // its X window.
      FILE* child = popen(
          "xdotool search --name '^EGO Occupancy - raw / inflated$' 2>/dev/null", "r");
      if (!child) {
        throw std::runtime_error("cannot inspect X windows");
      }
      char line[128];
      const bool retained = fgets(line, sizeof(line), child) != nullptr;
      pclose(child);
      if (retained) {
        throw std::runtime_error("display failure retained its X window/context");
      }
    } else {
      display.stop();
      if (!display.error().empty()) {
        throw std::runtime_error(display.error());
      }
      auto image = cv::imread(image_path.string());
      if (image.empty()) {
        throw std::runtime_error("render screenshot missing");
      }
      const auto pixel = image.at<cv::Vec3b>(650, 1100);
      if (cv::norm(pixel, cv::Vec3b(50, 100, 200), cv::NORM_INF) > 3) {
        throw std::runtime_error("RGB texture modulated by preceding 3D draw color");
      }
    }

    display.stop();
    std::filesystem::remove_all(root);
    std::cout << (cleanup ? "failure destroys owning-thread window"
                          : "current-frame RGB colors preserved")
              << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
