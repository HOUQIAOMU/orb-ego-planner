#include <ego_occupancy/display.h>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <opencv2/imgcodecs.hpp>

namespace {
using namespace ego_occupancy;

bool HasBlendedLegendText(const cv::Mat& image) {
  // The first "0" in the fixed near label has a transparent interior. Counting
  // antialiased rectangle edges alone would also accept unblended glyph quads.
  int transparent_pixels = 0;
  for (int y = image.rows - 23; y < image.rows - 15; ++y) {
    for (int x = 265; x < 272; ++x) {
      const auto pixel = image.at<cv::Vec3b>(y, x);
      if (std::max({pixel[0], pixel[1], pixel[2]}) < 60) {
        ++transparent_pixels;
      }
    }
  }
  return transparent_pixels >= 8;
}

cv::Point FindDepthPixel(const cv::Mat& image) {
  for (int y = 40; y < image.rows - 100; ++y) {
    for (int x = 250; x < image.cols - 20; ++x) {
      if (x >= 840 && y >= 480) {
        continue;  // Current Frame is not part of the map color contract.
      }
      const auto pixel = image.at<cv::Vec3b>(y, x);
      // Z=2.05m in [0.1,4.5] gives approximately RGB(57,255,0).
      if (pixel[0] < 8 && pixel[1] > 245 && pixel[2] > 35 && pixel[2] < 90) {
        return {x, y};
      }
    }
  }
  return {-1, -1};
}
}  // namespace

int main() {
  if (!std::getenv("EGO_TEST_DESKTOP") || !std::getenv("DISPLAY")) {
    return 77;
  }
  try {
    const auto root = std::filesystem::temp_directory_path() /
                      ("ego-depth-render-" + std::to_string(getpid()));
    std::filesystem::create_directory(root);
    const auto screenshot = root / "depth.png";
    ego_pose_bridge::Initialize(8800);
    static int map_identity = 0;
    const float transform[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                  0, 0, 1, 0, 0, 0, 0, 1};
    ego_pose_bridge::BeginFrame(1.0);
    ego_pose_bridge::Capture(1, 1.0, &map_identity, transform, 2, true);
    const auto pose = ego_pose_bridge::Last();
    Config config;
    config.size = Vec3(4, 4, 8);
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
    for (int i = 0; i < 3; ++i) {
      RayObservation observation;
      observation.source = {pose.session, pose.map_token, pose.revision};
      observation.timestamp_sec = 1.0 + i * 0.1;
      observation.frame_index = i;
      observation.received = Clock::now();
      observation.rays = {{Vec3(0.3, 0.2, 2.0), true}};
      mapper.integrate(observation);
    }
    OccupancyDisplay display(mapper.reader(), camera, screenshot, 1.0);
    display.publishCurrentFrame(cv::Mat(480, 640, CV_8UC3, cv::Scalar(50, 100, 200)),
                                pose, 1, Clock::now(), true);
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    cv::Point point(-1, -1);
    bool gray_after_expiry = false;
    while (Clock::now() < deadline && !display.closed()) {
      if (std::filesystem::exists(screenshot)) {
        const auto image = cv::imread(screenshot.string());
        if (!image.empty()) {
          if (point.x < 0) {
            point = FindDepthPixel(image);
            if (point.x >= 0 && !HasBlendedLegendText(image)) {
              throw std::runtime_error("legend glyph alpha blending is missing");
            }
          } else if (cv::norm(image.at<cv::Vec3b>(point), cv::Vec3b(102, 102, 102),
                              cv::NORM_INF) <= 3) {
            gray_after_expiry = true;
            break;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    display.stop();
    if (!display.error().empty()) {
      throw std::runtime_error(display.error());
    }
    if (point.x < 0 || !gray_after_expiry) {
      throw std::runtime_error("expected actual optical-depth color then gray on pose expiry");
    }
    std::filesystem::remove_all(root);
    std::cout << "batch depth rendering and pose expiry passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
