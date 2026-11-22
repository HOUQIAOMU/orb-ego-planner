#include <ego_occupancy/io.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

using namespace ego_occupancy;

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      throw std::invalid_argument("Usage: ego-map-demo NEW_OUTPUT_DIRECTORY");
    }

    const std::filesystem::path directory(argv[1]);
    if (std::filesystem::exists(directory)) {
      throw std::runtime_error("output already exists");
    }
    std::filesystem::create_directories(directory);

    Config config;
    config.size = Vec3(4, 4, 4);
    config.resolution = 0.05;
    config.inflation = Vec3(0.1, 0.1, 0.1);

    OccupancyMapper mapper(config);
    const SourceStamp source{1, 1, 0};
    mapper.reset(source, Vec3::Zero());

    for (int frame = 0; frame < 8; ++frame) {
      RayObservation observation;
      observation.source = source;
      observation.frame_index = frame;
      observation.timestamp_sec = frame * 0.1;
      observation.received = Clock::now();
      observation.origin = Vec3(0.025, 0.025, -1.025);

      for (int x = -20; x <= 20; ++x) {
        for (int y = -20; y <= 20; ++y) {
          Vec3 target(0.025 + x * 0.05, 0.025 + y * 0.05, 1.025);
          if (std::abs(x) < 8 && std::abs(y) < 8) {
            target = observation.origin + (target - observation.origin) * 1.3;
          }
          observation.rays.push_back({target, true});
        }
      }

      const auto result = mapper.integrate(observation);
      if (!result.accepted) {
        throw std::runtime_error(result.reason);
      }
    }

    const auto snapshot = mapper.reader().acquire();
    if (!snapshot) {
      throw std::runtime_error("demo map not available");
    }
    saveSnapshot(*snapshot, directory / "map.egovox");
    exportPly(*snapshot, directory / "occupied.ply");
    exportPly(*snapshot, directory / "inflated.ply", true);

    const auto opening = snapshot->query(Vec3(0.025, 0.025, 1.025));
    const auto rim = snapshot->query(Vec3(0.625, 0.025, 1.025));
    std::cout << "SYNTHETIC_OPENING center_state=" << static_cast<int>(opening.state)
              << " center_inflated=" << opening.inflated
              << " rim_state=" << static_cast<int>(rim.state)
              << " voxels=" << snapshot->voxelCount() << '\n';

    if (opening.state != VoxelState::Free || opening.inflated ||
        rim.state != VoxelState::Occupied) {
      throw std::runtime_error("known opening geometry failed");
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
