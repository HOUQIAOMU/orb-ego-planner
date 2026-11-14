#include <ego_occupancy/io.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace ego_occupancy;

int main(int argc, char** argv) {
  try {
    if (argc != 2 && argc != 5) {
      throw std::invalid_argument("Usage: ego-map-inspect MAP.egovox [x_m y_m z_m]");
    }

    const auto snapshot = loadSnapshot(argv[1]);
    std::size_t count[3] = {};
    std::size_t inflated = 0;
    snapshot->forEach(Index::Zero(), snapshot->dimensions(),
                      [&count, &inflated](const Index&, const Vec3&, VoxelQuery query) {
                        ++count[static_cast<int>(query.state)];
                        if (query.inflated) {
                          ++inflated;
                        }
                      });

    std::cout << "MAP frame=" << snapshot->metadata().frame_id
              << " epoch=" << snapshot->metadata().epoch
              << " sequence=" << snapshot->metadata().sequence
              << " resolution_m=" << snapshot->resolution()
              << " dimensions=" << snapshot->dimensions().transpose()
              << " unknown=" << count[0] << " free=" << count[1]
              << " occupied=" << count[2] << " inflated=" << inflated
              << " offline_snapshot=1\n";

    if (argc == 5) {
      Vec3 point;
      for (int i = 0; i < 3; ++i) {
        const std::string text(argv[i + 2]);
        std::size_t consumed = 0;
        point[i] = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(point[i])) {
          throw std::invalid_argument("invalid query coordinate");
        }
      }

      const auto query = snapshot->query(point);
      std::cout << "QUERY state=" << static_cast<int>(query.state)
                << " inflated=" << query.inflated
                << " blocked=" << snapshot->isBlocked(point) << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
