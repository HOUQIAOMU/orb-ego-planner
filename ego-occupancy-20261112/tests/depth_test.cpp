#include <ego_occupancy/depth.h>

#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace ego_occupancy;

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      throw std::runtime_error(#condition);                                    \
    }                                                                          \
  } while (false)

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
    CameraModel camera;
    camera.width = 3;
    camera.height = 1;
    camera.fx = 2;
    camera.fy = 2;
    camera.cx = 1;
    camera.cy = 0;
    camera.depth_scale = 0.001;
    camera.stride = 1;

    DepthProjector projector(camera);
    cv::Mat depth(1, 3, CV_16UC1);
    depth.at<std::uint16_t>(0, 0) = 1000;
    depth.at<std::uint16_t>(0, 1) = 0;
    depth.at<std::uint16_t>(0, 2) = 2000;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Vec3(1, 2, 3);

    auto observation =
        projector.project(depth, pose, SourceStamp{1, 2, 3}, 7, 4.0, Clock::now());
    CHECK(observation.rays.size() == 2);
    CHECK(observation.frame_index == 7);
    CHECK(observation.source.revision == 3);
    CHECK((observation.rays[0].endpoint - Vec3(0.5, 2, 4)).norm() < 1e-10);
    CHECK((observation.rays[1].endpoint - Vec3(2, 2, 5)).norm() < 1e-10);

    cv::Mat float_depth(1, 3, CV_32FC1);
    float_depth.at<float>(0, 0) = 1;
    float_depth.at<float>(0, 1) = std::numeric_limits<float>::quiet_NaN();
    float_depth.at<float>(0, 2) = -1;
    observation =
        projector.project(float_depth, pose, {1, 2, 3}, 8, 4.1, Clock::now());
    CHECK(observation.rays.size() == 1);
    CHECK((observation.rays[0].endpoint - Vec3(0.5, 2, 4)).norm() < 1e-10);

    ExpectFailure([&] {
      projector.project(cv::Mat(2, 3, CV_16UC1), pose, {1, 2, 3}, 9, 4.2, Clock::now());
    });

    auto invalid_pose = pose;
    invalid_pose.matrix()(0, 0) = 2;
    ExpectFailure([&] {
      projector.project(depth, invalid_pose, {1, 2, 3}, 9, 4.2, Clock::now());
    });

    camera.distortion_model = "unsupported";
    ExpectFailure([&] { DepthProjector invalid(camera); });

    camera.distortion_model = "brown_conrady";
    camera.distortion[0] = 0.1;
    DepthProjector brown(camera);
    observation = brown.project(depth, pose, {1, 2, 3}, 9, 4.2, Clock::now());
    CHECK(observation.rays[0].endpoint.x() > 0.5);
    CHECK(observation.rays[0].endpoint.x() < 0.6);

    const auto file = std::filesystem::temp_directory_path() /
                      ("ego-camera-test-" + std::to_string(getpid()) + ".yaml");
    {
      std::ofstream output(file);
      output << "%YAML:1.0\n"
                "Camera.type: \"Rectified\"\n"
                "Camera.RGB: 0\n"
                "Camera.width: 3\n"
                "Camera.height: 1\n"
                "Camera1.fx: 2.\n"
                "Camera1.fy: 2.\n"
                "Camera1.cx: 1.\n"
                "Camera1.cy: 0.\n"
                "RGBD.DepthMapFactor: 1000.\n";
    }
    ExpectFailure([&] { readCameraModel(file.string()); });
    std::filesystem::remove(file);

    std::cout << "metric depth, invalid pixels, pose and calibrated rays passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
