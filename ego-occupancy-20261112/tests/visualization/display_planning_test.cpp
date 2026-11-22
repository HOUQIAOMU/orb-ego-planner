#include <ego_occupancy/display.h>
#include <unistd.h>

#include <cstdlib>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <thread>
int main(int argc, char** argv) {
  try {
    if (!std::getenv("EGO_TEST_DESKTOP")) return 77;
    using namespace ego_occupancy;
    const auto file = std::filesystem::temp_directory_path() /
                      ("ego-planning-render-" + std::to_string(getpid()) + ".png");
    Config c;
    c.size = {2, 2, 4};
    c.resolution = .1;
    OccupancyMapper mapper(c);
    mapper.reset({1, 2, 3}, Vec3::Zero());
    CameraModel camera;
    camera.width = 640;
    camera.height = 480;
    camera.fx = camera.fy = 500;
    camera.cx = 320;
    camera.cy = 240;
    DisplaySettings settings;
    const bool gray = argc > 1 && std::string(argv[1]) == "gray";
    settings.planning_layer = [gray] {
      PlanningLayer p;
      p.has_task = true;
      p.ready = !gray;
      p.version = 1;
      p.status = "READY static preview";
      p.start = {-.6, -.4, 1};
      p.goal = {.6, -.4, 1};
      p.trajectory = {p.start, p.goal};
      return p;
    };
    OccupancyDisplay display(mapper.reader(), camera, file, .5, settings);
    auto deadline = Clock::now() + std::chrono::seconds(8);
    while (!std::filesystem::exists(file) && !display.closed() && Clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    display.stop();
    if (!display.error().empty()) throw std::runtime_error(display.error());
    auto image = cv::imread(file.string());
    if (image.empty()) throw std::runtime_error("planning screenshot missing");
    int cyan = 0;
    for (int y = 0; y < image.rows; ++y)
      for (int x = 240; x < image.cols; ++x) {
        auto p = image.at<cv::Vec3b>(y, x);
        if ((!gray && p[0] > 240 && p[1] > 240 && p[2] < 15) ||
            (gray && std::abs(int(p[0]) - 130) < 3 && std::abs(int(p[1]) - 130) < 3 &&
             std::abs(int(p[2]) - 130) < 3))
          ++cyan;
      }
    if (cyan < 20) throw std::runtime_error("trajectory cyan line missing from actual framebuffer");
    std::filesystem::remove(file);
    std::cout << "PASS planning layer actual framebuffer\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
