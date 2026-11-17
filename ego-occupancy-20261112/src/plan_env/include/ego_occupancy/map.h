#pragma once

#include <Eigen/Core>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ego_occupancy {

using Vec3 = Eigen::Vector3d;
using Index = Eigen::Vector3i;
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

enum class VoxelState : std::uint8_t {
  Unknown,
  Free,
  Occupied,
  Outside,
};

enum class UnknownPolicy {
  Block,
  Allow,
};

struct VoxelQuery {
  VoxelState state = VoxelState::Outside;
  bool inflated = false;
};

/// Identity of the source map and its correction revision.
///
/// A new session, a new map token, or a new revision invalidates every older
/// snapshot derived from the previous identity.
struct SourceStamp {
  std::uint64_t session = 0;
  std::uint64_t map = 0;
  std::uint64_t revision = 0;

  bool operator==(const SourceStamp& other) const {
    return session == other.session && map == other.map && revision == other.revision;
  }

  bool operator!=(const SourceStamp& other) const { return !(*this == other); }
};

struct Config {
  Vec3 size{8.0, 8.0, 4.0};
  double resolution = 0.05;
  Vec3 inflation{0.10, 0.10, 0.10};
  double p_hit = 0.70;
  double p_miss = 0.35;
  double p_min = 0.12;
  double p_max = 0.97;
  double p_free = 0.35;
  double p_occ = 0.80;
  double min_ray_length = 0.10;
  double max_ray_length = 4.5;
  double max_age_sec = 0.5;
  std::size_t max_voxels = 8'000'000;
  std::size_t max_rays = 1'000'000;
  std::size_t memory_budget_bytes = 512 * 1024 * 1024;
  std::string frame_id = "slam_local";
};

/// Allocation-free configuration check with the same rules as OccupancyMapper.
void validateConfig(const Config& config);

struct Ray {
  Vec3 endpoint;
  bool hit = true;
};

struct RayObservation {
  SourceStamp source;
  std::uint64_t frame_index = 0;
  double timestamp_sec = 0.0;
  TimePoint received{};
  Vec3 origin = Vec3::Zero();
  std::vector<Ray> rays;
};

struct Metadata {
  std::uint32_t schema_version = 1;
  SourceStamp source;
  std::uint64_t epoch = 0;
  std::uint64_t sequence = 0;
  std::uint64_t frame_index = 0;
  double timestamp_sec = 0.0;
  TimePoint received{};
  TimePoint published{};
  std::string frame_id;
};

struct UpdateResult {
  bool accepted = false;
  bool published = false;
  std::size_t valid_rays = 0;
  std::size_t updated_voxels = 0;
  double fuse_ms = 0.0;
  double inflate_ms = 0.0;
  double publish_ms = 0.0;
  std::string reason;
};

struct ReaderStatus {
  bool accepting = false;
  bool valid = false;
  std::uint64_t epoch = 0;
  std::string reason;
};

struct MapControl;
struct SnapshotIO;

/// Immutable, bounded view of the occupancy grid.
///
/// The mapper publishes snapshots by copying its current state into one of
/// three preallocated slots. A snapshot remains valid until its lease is
/// released; older leases never observe newer data.
class MapSnapshot {
 public:
  VoxelQuery query(const Vec3& point) const;
  VoxelQuery query(const Index& index) const;
  bool isBlocked(const Vec3& point, UnknownPolicy policy = UnknownPolicy::Block) const;

  std::optional<Index> worldToIndex(const Vec3& point) const;
  Vec3 indexToWorld(const Index& index) const;

  void forEach(
      const Index& minimum,
      const Index& maximum_exclusive,
      const std::function<void(const Index&, const Vec3&, VoxelQuery)>& callback) const;

  double resolution() const { return resolution_; }
  const Vec3& origin() const { return origin_; }
  const Index& dimensions() const { return dimensions_; }
  std::size_t voxelCount() const { return states_.size(); }
  const Metadata& metadata() const { return metadata_; }

 private:
  friend class OccupancyMapper;
  friend struct SnapshotIO;

  Vec3 origin_ = Vec3::Zero();
  Index dimensions_ = Index::Zero();
  double resolution_ = 0.0;
  Metadata metadata_;
  std::vector<std::uint8_t> states_;
  std::vector<std::uint8_t> inflated_;
};

class MapReader {
 public:
  std::shared_ptr<const MapSnapshot> acquire(TimePoint now = Clock::now()) const;
  std::shared_ptr<const MapSnapshot> latestForDisplay() const;
  bool isUsable(const std::shared_ptr<const MapSnapshot>& snapshot,
                TimePoint now = Clock::now()) const;
  ReaderStatus status() const;

 private:
  friend class OccupancyMapper;

  explicit MapReader(std::shared_ptr<MapControl> control) : control_(std::move(control)) {}

  std::shared_ptr<MapControl> control_;
};

/// Single writer for reset/integrate; invalidate is safe from a source observer thread.
class OccupancyMapper {
 public:
  explicit OccupancyMapper(const Config& config);
  ~OccupancyMapper();

  OccupancyMapper(const OccupancyMapper&) = delete;
  OccupancyMapper& operator=(const OccupancyMapper&) = delete;

  void reset(SourceStamp source, const Vec3& center);
  void invalidate(const std::string& reason);
  UpdateResult integrate(const RayObservation& observation, TimePoint now = Clock::now());

  MapReader reader() const;
  const Config& config() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// Minimal adapter matching the EGO-Planner grid-map collision query.
class EgoGridMapAdapter {
 public:
  explicit EgoGridMapAdapter(std::shared_ptr<const MapSnapshot> snapshot,
                             UnknownPolicy policy = UnknownPolicy::Block)
      : snapshot_(std::move(snapshot)), policy_(policy) {}

  double getResolution() const;
  int getInflateOccupancy(const Vec3& point) const;

 private:
  std::shared_ptr<const MapSnapshot> snapshot_;
  UnknownPolicy policy_;
};

}  // namespace ego_occupancy
