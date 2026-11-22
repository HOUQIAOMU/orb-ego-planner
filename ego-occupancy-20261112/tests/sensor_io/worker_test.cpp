#include <ego_occupancy/worker.h>

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

ego_pose_bridge::Record MakeFrame(std::uint64_t index) {
  static int map_identity = 0;
  float transform[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  ego_pose_bridge::BeginFrame(index * 0.1);
  ego_pose_bridge::Capture(index, index * 0.1, &map_identity, transform, 2, true);
  return ego_pose_bridge::Last();
}

int main() {
  try {
    ego_pose_bridge::Initialize(555);

    Config config;
    config.size = Vec3(2, 2, 4);
    config.resolution = 0.1;
    config.inflation = Vec3::Zero();

    CameraModel camera;
    camera.width = 3;
    camera.height = 1;
    camera.fx = camera.fy = 2;
    camera.cx = 1;
    camera.cy = 0;
    camera.depth_scale = 0.001;
    camera.stride = 1;

    MappingWorker worker(config, camera);
    cv::Mat depth(1, 3, CV_16UC1, cv::Scalar(1000));

    CHECK(worker.submit(depth, MakeFrame(1), 1, Clock::now()));
    CHECK(worker.waitFor(1, std::chrono::seconds(2)));
    auto first = worker.reader().acquire();
    CHECK(first);

    worker.trackingLost("test lost");
    CHECK(!worker.reader().isUsable(first));

    auto old = MakeFrame(2);
    {
      ego_pose_bridge::CorrectionScope correction("test loop");
    }
    CHECK(!worker.submit(depth, old, 2, Clock::now()));

    CHECK(worker.submit(depth, MakeFrame(3), 3, Clock::now()));
    CHECK(worker.waitFor(3, std::chrono::seconds(2)));
    auto second = worker.reader().acquire();
    CHECK(second && second->metadata().epoch != first->metadata().epoch);

    worker.stop();
    CHECK(!worker.reader().isUsable(second));
    CHECK(worker.reader().latestForDisplay());
    CHECK(worker.stats().fused == 2);

    std::cout << "worker pairing, correction invalidation, recovery and shutdown passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
