#include <ego_occupancy/display_color.h>
#include <ego_occupancy/io.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using namespace ego_occupancy;

void Measure(const std::vector<Vec3>& points, ColorMode mode) {
  std::vector<Rgb8> colors;
  std::vector<double> samples;
  Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 101; ++i) {
    camera.translation().z() = i * 0.001;
    const auto start = Clock::now();
    colorizePoints(points, mode, {0.1, 4.5}, false, camera, true, colors);
    const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    if (i != 0) {
      samples.push_back(elapsed);  // Exclude first allocation/table initialization.
    }
  }
  std::sort(samples.begin(), samples.end());
  std::cout << points.size() << ',' << static_cast<int>(mode) << ','
            << samples[50] << ',' << samples[95] << ','
            << colors.size() * sizeof(Rgb8) << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      throw std::invalid_argument("usage: occupancy-display-color-benchmark MAP.egovox");
    }
    const auto map = loadSnapshot(argv[1]);
    std::vector<Vec3> points;
    map->forEach(Index::Zero(), map->dimensions(),
                 [&points](const Index&, const Vec3& point, VoxelQuery query) {
                   if (query.state == VoxelState::Occupied) {
                     points.push_back(point);
                   }
                 });
    if (points.empty()) {
      throw std::runtime_error("benchmark requires occupied voxels");
    }
    std::cout << "points,mode,p50_ms,p95_ms,color_bytes\n";
    Measure(points, ColorMode::Flat);
    Measure(points, ColorMode::CameraDepth);
    const auto original = points;
    points.resize(100000);
    for (std::size_t i = 0; i < points.size(); ++i) {
      points[i] = original[i % original.size()];
    }
    Measure(points, ColorMode::Flat);
    Measure(points, ColorMode::CameraDepth);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
