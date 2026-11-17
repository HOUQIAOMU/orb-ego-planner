#include <ego_occupancy/io.h>

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
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
    const auto directory = std::filesystem::temp_directory_path() /
                           ("ego-io-test-" + std::to_string(getpid()));
    std::filesystem::create_directory(directory);

    Config config;
    config.size = Vec3(2, 2, 2);
    config.resolution = 0.25;
    config.inflation = Vec3::Zero();
    OccupancyMapper mapper(config);
    const SourceStamp source{2, 3, 4};
    mapper.reset(source, Vec3::Zero());

    RayObservation observation;
    observation.source = source;
    observation.origin = Vec3(0.125, 0.125, 0.125);
    observation.received = Clock::now();
    observation.rays = {Ray{Vec3(0.875, 0.125, 0.125), true}};
    mapper.integrate(observation, observation.received);

    observation.frame_index = 1;
    observation.timestamp_sec = 0.1;
    observation.received = Clock::now();
    mapper.integrate(observation, observation.received);
    auto snapshot = mapper.reader().acquire();
    CHECK(snapshot);

    const auto file = directory / "map.egovox";
    saveSnapshot(*snapshot, file);
    auto loaded = loadSnapshot(file);
    CHECK(loaded->metadata().source == source);
    CHECK(loaded->metadata().epoch == snapshot->metadata().epoch);
    CHECK(loaded->voxelCount() == 512);
    CHECK(loaded->query(Vec3(0.875, 0.125, 0.125)).state == VoxelState::Occupied);
    CHECK(loaded->query(Vec3(0.375, 0.125, 0.125)).state == VoxelState::Free);
    CHECK(loaded->query(Vec3(-0.875, 0, 0)).state == VoxelState::Unknown);

    std::size_t visited = 0;
    loaded->forEach(Index::Zero(), loaded->dimensions(),
                    [&snapshot, &visited](const Index& index, const Vec3&, VoxelQuery query) {
                      const auto baseline = snapshot->query(index);
                      CHECK(query.state == baseline.state &&
                            query.inflated == baseline.inflated);
                      ++visited;
                    });
    CHECK(visited == 512);

    ExpectFailure([&] { saveSnapshot(*snapshot, file); });
    ExpectFailure([&] { loadSnapshot(file, 10); });

    const auto broken = directory / "broken.egovox";
    std::filesystem::copy_file(file, broken);
    std::filesystem::resize_file(broken, 20);
    ExpectFailure([&] { loadSnapshot(broken); });

    std::filesystem::copy_file(file, broken,
                               std::filesystem::copy_options::overwrite_existing);
    {
      std::fstream stream(broken, std::ios::binary | std::ios::in | std::ios::out);
      stream.seekp(12);
      const char byte = 127;
      stream.write(&byte, 1);
    }
    ExpectFailure([&] { loadSnapshot(broken); });

    exportPly(*loaded, directory / "occupied.ply", false);
    CHECK(std::filesystem::file_size(directory / "occupied.ply") > 100);

    Config tiny;
    tiny.size = Vec3::Ones();
    tiny.resolution = 1;
    tiny.inflation = Vec3::Zero();
    tiny.frame_id = "x";
    OccupancyMapper single(tiny);
    single.reset(source, Vec3::Zero());
    observation.origin = Vec3(0.1, 0.1, 0.1);
    observation.rays = {Ray{Vec3(0.4, 0.1, 0.1), true}};
    observation.received = Clock::now();
    single.integrate(observation, observation.received);
    auto single_snapshot = single.reader().acquire();
    CHECK(single_snapshot && single_snapshot->voxelCount() == 1);
    saveSnapshot(*single_snapshot, directory / "tiny.egovox");
    CHECK(loadSnapshot(directory / "tiny.egovox")->voxelCount() == 1);

    std::filesystem::remove_all(directory);
    std::cout << "snapshot roundtrip, bounded parsing, corruption and export passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
