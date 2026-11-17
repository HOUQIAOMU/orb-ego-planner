#include <ego_occupancy/current_frame.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>

using namespace ego_occupancy;

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      throw std::runtime_error(#condition);                                    \
    }                                                                          \
  } while (false)

int main() {
  try {
    ego_pose_bridge::Initialize(6789);

    CameraModel camera;
    camera.width = 4;
    camera.height = 2;
    camera.fx = camera.fy = 2;
    camera.cx = 1.5;
    camera.cy = 0.5;
    camera.depth_scale = 0.001;

    // Rotate +90 degrees about optical Z, then translate (1, 2, 3).
    float matrix[16] = {0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1};
    static int map_identity = 0;
    ego_pose_bridge::BeginFrame(10.0);
    ego_pose_bridge::Capture(40, 10.0, &map_identity, matrix, 2, true);
    auto record = ego_pose_bridge::Last();
    const auto received = Clock::now();

    CurrentFrameBuffer frames(camera);
    cv::Mat storage(2, 8, CV_8UC3, cv::Scalar(11, 22, 33));
    auto roi = storage(cv::Rect(1, 0, 4, 2));
    frames.publish(roi, record, 7, received, true);
    auto first = frames.latest();
    roi.setTo(cv::Scalar(1, 2, 3));
    CHECK(first && first->capture_index == 7 && first->bgr.isContinuous());
    CHECK(first->bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(11, 22, 33));

    Config config;
    config.size = Vec3(4, 4, 4);
    config.resolution = 0.1;
    OccupancyMapper mapper(config);
    const SourceStamp source{record.session, record.map_token, record.revision};
    mapper.reset(source, Vec3(1, 2, 3));

    RayObservation observation;
    observation.source = source;
    observation.timestamp_sec = 10.0;
    observation.received = received;
    observation.origin = Vec3(1, 2, 3);
    observation.rays = {Ray{Vec3(1, 2, 4), true}};
    CHECK(mapper.integrate(observation).accepted);

    auto snapshot = mapper.reader().latestForDisplay();
    CHECK(snapshot);
    auto status = mapper.reader().status();
    auto pose = cameraPoseForMap(*first, *snapshot, status, received, 0.5);
    CHECK(pose && pose->translation().isApprox(Vec3(1, 2, 3)));

    auto corners = cameraFrustum(*pose, camera, 0.2);
    CHECK(corners[0].isApprox(Vec3(1, 2, 3)));
    CHECK(corners[1].isApprox(Vec3(1.05, 1.85, 3.2)));
    CHECK(corners[3].isApprox(Vec3(0.95, 2.15, 3.2)));
    CHECK(!cameraPoseForMap(*first, *snapshot, status, received + std::chrono::seconds(1),
                            0.5));
    CHECK(!cameraPoseForMap(*first, *snapshot, status, received - std::chrono::seconds(1),
                            0.5));

    auto reset = status;
    ++reset.epoch;
    CHECK(!cameraPoseForMap(*first, *snapshot, reset, received, 0.5));
    reset = status;
    reset.valid = false;
    CHECK(!cameraPoseForMap(*first, *snapshot, reset, received, 0.5));

    auto wrong = *first;
    ++wrong.pose.map_token;
    CHECK(!cameraPoseForMap(wrong, *snapshot, status, received, 0.5));

    // Simulate a frame arriving while the renderer prepares voxel geometry.
    // The sample must obtain its clock after obtaining that published frame.
    frames.publish(roi, record, 8, Clock::now(), true);
    const auto sample = frames.sample();
    CHECK(sample.frame && sample.sampled_at >= sample.frame->received);
    CHECK(cameraPoseForMap(*sample.frame, *snapshot, status, sample.sampled_at, 0.5));

    frames.publish(roi, record, 8, Clock::now() - std::chrono::seconds(1), true);
    const auto expired = frames.sample();
    CHECK(!cameraPoseForMap(*expired.frame, *snapshot, status, expired.sampled_at, 0.5));

    // RGB remains current during tracking loss; old displayed image stays immutable.
    record.valid = false;
    record.tracking_state = 3;
    cv::Mat lost(2, 4, CV_8UC3, cv::Scalar(80, 90, 100));
    frames.publish(lost, record, 8, received, false);
    auto second = frames.latest();
    CHECK(second->capture_index == 8 &&
          second->bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(80, 90, 100));
    CHECK(!cameraPoseForMap(*second, *snapshot, status, received, 0.5));
    CHECK(first->capture_index == 7 &&
          first->bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(11, 22, 33));

    bool rejected = false;
    try {
      frames.publish(cv::Mat(1, 4, CV_8UC3), record, 9, received, false);
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    CHECK(rejected && frames.latest()->capture_index == 8);

    {
      ego_pose_bridge::CorrectionScope correction("display test reset");
    }
    CHECK(!cameraPoseForMap(*first, *snapshot, status, received, 0.5));

    std::cout << "owned RGB, lost-tracking display, pose age/source/epoch and optical "
                 "geometry passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
