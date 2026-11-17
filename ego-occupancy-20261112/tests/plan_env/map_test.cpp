#include <ego_occupancy/map.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

using namespace ego_occupancy;

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      throw std::runtime_error(std::string(__func__) + ":" +                  \
                               std::to_string(__LINE__) + " " #condition);     \
    }                                                                          \
  } while (false)

template <class Function>
void Throws(Function function) {
  bool caught = false;
  try {
    function();
  } catch (const std::exception&) {
    caught = true;
  }
  CHECK(caught);
}

const SourceStamp kSource{1, 1, 0};
const TimePoint kStart = TimePoint{} + std::chrono::seconds(100);

Config MakeConfig() {
  Config config;
  config.size = Vec3(4, 4, 4);
  config.resolution = 0.25;
  config.inflation = Vec3::Zero();
  return config;
}

RayObservation MakeObservation(std::uint64_t frame, const Vec3& endpoint, bool hit = true) {
  RayObservation observation;
  observation.source = kSource;
  observation.frame_index = frame;
  observation.timestamp_sec = frame * 0.01;
  observation.received = kStart + std::chrono::milliseconds(frame);
  observation.origin = Vec3(0.125, 0.125, 0.125);
  observation.rays.push_back({endpoint, hit});
  return observation;
}

void GeometryAndUnknown() {
  OccupancyMapper mapper(MakeConfig());
  mapper.reset(kSource, Vec3::Zero());
  auto reader = mapper.reader();

  CHECK(!reader.acquire(kStart));
  auto observation = MakeObservation(0, Vec3(1.125, 0.125, 0.125));
  CHECK(mapper.integrate(observation, kStart).accepted);

  auto snapshot = reader.acquire(kStart);
  CHECK(snapshot);
  CHECK(snapshot->query(Vec3(-1.99, 0, 0)).state == VoxelState::Unknown);
  CHECK(snapshot->query(Vec3(2, 0, 0)).state == VoxelState::Outside);
  CHECK(snapshot->query(Vec3(-2, 0, 0)).state == VoxelState::Unknown);
  CHECK(snapshot->worldToIndex(Vec3(-1.99, -1.99, -1.99)).value() == Index(0, 0, 0));
  CHECK(!snapshot->worldToIndex(Vec3(std::numeric_limits<double>::quiet_NaN(), 0, 0)));
  CHECK((snapshot->indexToWorld(Index(8, 8, 8)) - Vec3(0.125, 0.125, 0.125)).norm() <
        1e-12);
  CHECK(snapshot->isBlocked(Vec3(-1, 0, 0)));
  CHECK(!snapshot->isBlocked(Vec3(-1, 0, 0), UnknownPolicy::Allow));
  CHECK(snapshot->isBlocked(Vec3(2, 0, 0), UnknownPolicy::Allow));
  Throws([&] { snapshot->indexToWorld(Index(16, 0, 0)); });
}

void HitMissAndInflationRemoval() {
  auto config = MakeConfig();
  config.inflation = Vec3(0.25, 0.25, 0.25);
  OccupancyMapper mapper(config);
  mapper.reset(kSource, Vec3::Zero());

  for (int frame = 0; frame < 8; ++frame) {
    auto observation = MakeObservation(frame, Vec3(1.125, 0.125, 0.125));
    CHECK(mapper.integrate(observation, observation.received).accepted);
  }

  auto snapshot = mapper.reader().acquire(kStart + std::chrono::milliseconds(8));
  CHECK(snapshot);
  CHECK(snapshot->query(Vec3(0.625, 0.125, 0.125)).state == VoxelState::Free);
  CHECK(snapshot->query(Vec3(1.125, 0.125, 0.125)).state == VoxelState::Occupied);
  CHECK(snapshot->query(Vec3(1.375, 0.125, 0.125)).inflated);
  EgoGridMapAdapter adapter(snapshot);
  CHECK(adapter.getInflateOccupancy(Vec3(1.375, 0.125, 0.125)) == 1);
  CHECK(adapter.getResolution() == 0.25);

  for (int frame = 8; frame < 24; ++frame) {
    auto observation = MakeObservation(frame, Vec3(1.875, 0.125, 0.125));
    CHECK(mapper.integrate(observation, observation.received).accepted);
  }

  auto next = mapper.reader().acquire(kStart + std::chrono::milliseconds(24));
  CHECK(next);
  CHECK(next->query(Vec3(1.125, 0.125, 0.125)).state == VoxelState::Free);
  CHECK(!next->query(Vec3(1.125, 0.125, 0.125)).inflated);
  CHECK(snapshot->query(Vec3(1.125, 0.125, 0.125)).state == VoxelState::Occupied);
}

void OneFrameVotesAndClipping() {
  auto config = MakeConfig();
  config.max_ray_length = 1.0;
  OccupancyMapper mapper(config);
  mapper.reset(kSource, Vec3::Zero());

  auto observation = MakeObservation(0, Vec3(4.125, 0.125, 0.125));
  observation.rays.resize(100, observation.rays[0]);
  CHECK(mapper.integrate(observation, kStart).accepted);

  auto snapshot = mapper.reader().acquire(kStart);
  CHECK(snapshot);
  CHECK(snapshot->query(Vec3(0.875, 0.125, 0.125)).state == VoxelState::Free);
  CHECK(snapshot->query(Vec3(1.125, 0.125, 0.125)).state != VoxelState::Occupied);

  mapper.reset(kSource, Vec3::Zero());
  observation = MakeObservation(1, Vec3(0.875, 0.125, 0.125));
  observation.rays.resize(100, observation.rays[0]);
  mapper.integrate(observation, observation.received);
  snapshot = mapper.reader().acquire(observation.received);
  CHECK(snapshot->query(Vec3(0.875, 0.125, 0.125)).state == VoxelState::Unknown);

  observation = MakeObservation(2, Vec3(0.875, 0.125, 0.125));
  mapper.integrate(observation, observation.received);
  snapshot = mapper.reader().acquire(observation.received);
  CHECK(snapshot->query(Vec3(0.875, 0.125, 0.125)).state == VoxelState::Occupied);
}

void DiagonalNegativeAndEdges() {
  auto config = MakeConfig();
  config.max_ray_length = 10;
  OccupancyMapper mapper(config);
  mapper.reset(kSource, Vec3::Zero());

  auto observation = MakeObservation(0, Vec3(-1.625, -1.625, -1.625));
  mapper.integrate(observation, observation.received);
  auto snapshot = mapper.reader().acquire(observation.received);
  CHECK(snapshot->query(Vec3(-0.625, -0.625, -0.625)).state == VoxelState::Free);

  observation = MakeObservation(1, Vec3(-1.625, -1.625, -1.625));
  mapper.integrate(observation, observation.received);
  snapshot = mapper.reader().acquire(observation.received);
  CHECK(snapshot->query(Vec3(-1.625, -1.625, -1.625)).state == VoxelState::Occupied);

  observation = MakeObservation(2, Vec3(5, 0.125, 0.125));
  mapper.integrate(observation, observation.received);
  snapshot = mapper.reader().acquire(observation.received);
  CHECK(snapshot->query(Vec3(1.875, 0.125, 0.125)).state == VoxelState::Free);
}

void BoundedSnapshotsAndInvalidation() {
  OccupancyMapper mapper(MakeConfig());
  mapper.reset(kSource, Vec3::Zero());
  auto reader = mapper.reader();

  auto observation = MakeObservation(0, Vec3(1.125, 0.125, 0.125));
  CHECK(mapper.integrate(observation, observation.received).published);
  auto first = reader.acquire(observation.received);

  observation = MakeObservation(1, Vec3(1.125, 0.125, 0.125));
  CHECK(mapper.integrate(observation, observation.received).published);
  auto second = reader.acquire(observation.received);

  observation = MakeObservation(2, Vec3(1.125, 0.125, 0.125));
  CHECK(mapper.integrate(observation, observation.received).published);
  auto third = reader.acquire(observation.received);

  observation = MakeObservation(3, Vec3(1.125, 0.125, 0.125));
  auto skipped = mapper.integrate(observation, observation.received);
  CHECK(skipped.accepted && !skipped.published);
  CHECK(first->query(Vec3(1.125, 0.125, 0.125)).state == VoxelState::Unknown);

  std::thread invalidator([&] { mapper.invalidate("source correction"); });
  invalidator.join();
  CHECK(!reader.isUsable(third, observation.received));
  CHECK(!reader.acquire(observation.received));

  SourceStamp other{1, 2, 0};
  mapper.reset(other, Vec3::Zero());
  CHECK(!mapper.integrate(MakeObservation(4, Vec3(1, 0, 0)), observation.received).accepted);
  CHECK(!reader.isUsable(first, observation.received));
  first.reset();
  second.reset();
  third.reset();

  observation = MakeObservation(5, Vec3(1.125, 0.125, 0.125));
  observation.source = other;
  CHECK(mapper.integrate(observation, observation.received).published);
  CHECK(reader.acquire(observation.received));
  CHECK(!reader.acquire(observation.received + std::chrono::seconds(1)));
  CHECK(!mapper.integrate(observation, observation.received).accepted);
}

void RejectBadConfigurationAndInput() {
  auto config = MakeConfig();
  config.resolution = 0;
  Throws([&] { OccupancyMapper mapper(config); });

  config = MakeConfig();
  config.size.x() = 1e50;
  Throws([&] { OccupancyMapper mapper(config); });

  config = MakeConfig();
  config.p_free = 0.9;
  Throws([&] { OccupancyMapper mapper(config); });

  OccupancyMapper mapper(MakeConfig());
  mapper.reset(kSource, Vec3::Zero());

  auto observation = MakeObservation(0, Vec3(1, 0, 0));
  observation.timestamp_sec = std::numeric_limits<double>::quiet_NaN();
  CHECK(!mapper.integrate(observation, kStart).accepted);

  observation = MakeObservation(0, Vec3(1, 0, 0));
  CHECK(!mapper.integrate(observation, kStart + std::chrono::seconds(1)).accepted);
  CHECK(!mapper.reader().acquire(kStart + std::chrono::seconds(1)));
}

void WeakSnapshotCannotResurrectRecycledStorage() {
  OccupancyMapper mapper(MakeConfig());
  mapper.reset(kSource, Vec3::Zero());
  auto observation = MakeObservation(0, Vec3(1.125, 0.125, 0.125));
  mapper.integrate(observation, observation.received);

  auto first = mapper.reader().acquire(observation.received);
  std::weak_ptr<const MapSnapshot> old = first;
  first.reset();

  observation = MakeObservation(1, Vec3(1.125, 0.125, 0.125));
  CHECK(mapper.integrate(observation, observation.received).published);
  CHECK(old.expired());
  CHECK(!old.lock());
}

int main() {
  try {
    GeometryAndUnknown();
    HitMissAndInflationRemoval();
    OneFrameVotesAndClipping();
    DiagonalNegativeAndEdges();
    BoundedSnapshotsAndInvalidation();
    RejectBadConfigurationAndInput();
    WeakSnapshotCannotResurrectRecycledStorage();
    std::cout << "7 voxel contract groups passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
