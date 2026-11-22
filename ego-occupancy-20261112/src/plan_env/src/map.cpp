#include <ego_occupancy/map.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace ego_occupancy {
namespace {

bool InBounds(const Index& index, const Index& dimensions) {
  return (index.array() >= 0).all() && (index.array() < dimensions.array()).all();
}

std::size_t Address(const Index& index, const Index& dimensions) {
  return (static_cast<std::size_t>(index.x()) * dimensions.y() + index.y()) *
             dimensions.z() +
         index.z();
}

double Logit(double probability) {
  return std::log(probability / (1.0 - probability));
}

double Milliseconds(TimePoint begin, TimePoint end) {
  return std::chrono::duration<double, std::milli>(end - begin).count();
}

std::optional<Index> IndexOf(const Vec3& point,
                             const Vec3& origin,
                             const Index& dimensions,
                             double resolution) {
  if (!point.allFinite()) {
    return std::nullopt;
  }

  const Vec3 grid = (point - origin) / resolution;
  if (!(grid.array() >= 0).all() || !(grid.array() < dimensions.cast<double>().array()).all()) {
    return std::nullopt;
  }

  return grid.array().floor().cast<int>().matrix();
}

struct Layout {
  Index dimensions;
  Index inflation_steps;
  std::size_t voxel_count = 0;
};

Layout ValidatedLayout(const Config& config) {
  const auto finite = [](double value) { return std::isfinite(value); };

  if (!config.size.allFinite() || !(config.size.array() > 0).all() ||
      !finite(config.resolution) || config.resolution <= 0 || !config.inflation.allFinite() ||
      (config.inflation.array() < 0).any() || config.frame_id.empty() ||
      config.frame_id.size() > 256 || !finite(config.min_ray_length) ||
      !finite(config.max_ray_length) || config.min_ray_length < 0 ||
      config.max_ray_length <= config.min_ray_length || !finite(config.max_age_sec) ||
      config.max_age_sec <= 0 || config.max_rays == 0 || config.max_rays > 100'000'000 ||
      !(0 < config.p_min && config.p_min <= config.p_free && config.p_free < 0.5 &&
        0.5 < config.p_occ && config.p_occ <= config.p_max && config.p_max < 1) ||
      !(0.5 < config.p_hit && config.p_hit < 1 && 0 < config.p_miss &&
        config.p_miss < 0.5)) {
    throw std::invalid_argument("invalid occupancy configuration");
  }

  const Vec3 dimensions = (config.size / config.resolution).array().ceil();
  if (!dimensions.allFinite() || (dimensions.array() < 1).any() ||
      (dimensions.array() > std::numeric_limits<int>::max()).any()) {
    throw std::invalid_argument("grid dimension overflow");
  }

  Layout layout;
  layout.dimensions = dimensions.cast<int>();
  layout.voxel_count = 1;
  for (int axis = 0; axis < 3; ++axis) {
    if (static_cast<std::size_t>(layout.dimensions[axis]) >
        config.max_voxels / layout.voxel_count) {
      throw std::invalid_argument("voxel count exceeds configured limit");
    }
    layout.voxel_count *= layout.dimensions[axis];
  }

  constexpr std::size_t kBytesPerVoxel = 8 + 3 + 4 + 4 + sizeof(std::size_t) + 6;
  if (layout.voxel_count > config.memory_budget_bytes / kBytesPerVoxel) {
    throw std::invalid_argument("grid exceeds memory budget");
  }

  const Vec3 inflation = (config.inflation / config.resolution).array().ceil();
  if (!inflation.allFinite() ||
      (inflation.array() > layout.dimensions.cast<double>().array()).any()) {
    throw std::invalid_argument("inflation exceeds map dimensions");
  }
  layout.inflation_steps = inflation.cast<int>();
  return layout;
}

}  // namespace

void validateConfig(const Config& config) { (void)ValidatedLayout(config); }

struct MapControl {
  mutable std::mutex mutex;
  ReaderStatus status;
  double max_age = 0.0;
  SourceStamp source;
  std::shared_ptr<const MapSnapshot> latest;

  bool Usable(const std::shared_ptr<const MapSnapshot>& snapshot, TimePoint now) const {
    return snapshot && status.accepting && status.valid &&
           snapshot->metadata().epoch == status.epoch &&
           snapshot->metadata().source == source && now >= snapshot->metadata().received &&
           std::chrono::duration<double>(now - snapshot->metadata().received).count() <=
               max_age;
  }
};

VoxelQuery MapSnapshot::query(const Vec3& point) const {
  const auto index = worldToIndex(point);
  return index ? query(*index) : VoxelQuery{};
}

VoxelQuery MapSnapshot::query(const Index& index) const {
  if (!InBounds(index, dimensions_)) {
    return {};
  }

  const auto offset = Address(index, dimensions_);
  return {static_cast<VoxelState>(states_[offset]), inflated_[offset] != 0};
}

bool MapSnapshot::isBlocked(const Vec3& point, UnknownPolicy policy) const {
  const VoxelQuery result = query(point);
  return result.state == VoxelState::Outside || result.state == VoxelState::Occupied ||
         result.inflated ||
         (result.state == VoxelState::Unknown && policy == UnknownPolicy::Block);
}

std::optional<Index> MapSnapshot::worldToIndex(const Vec3& point) const {
  return IndexOf(point, origin_, dimensions_, resolution_);
}

Vec3 MapSnapshot::indexToWorld(const Index& index) const {
  if (!InBounds(index, dimensions_)) {
    throw std::out_of_range("voxel index outside map");
  }
  return origin_ + (index.cast<double>() + Vec3::Constant(0.5)) * resolution_;
}

void MapSnapshot::forEach(
    const Index& minimum,
    const Index& maximum_exclusive,
    const std::function<void(const Index&, const Vec3&, VoxelQuery)>& callback) const {
  if ((minimum.array() < 0).any() || (maximum_exclusive.array() < minimum.array()).any() ||
      (maximum_exclusive.array() > dimensions_.array()).any()) {
    throw std::out_of_range("invalid voxel traversal bounds");
  }

  for (int x = minimum.x(); x < maximum_exclusive.x(); ++x) {
    for (int y = minimum.y(); y < maximum_exclusive.y(); ++y) {
      for (int z = minimum.z(); z < maximum_exclusive.z(); ++z) {
        const Index index(x, y, z);
        callback(index, indexToWorld(index), query(index));
      }
    }
  }
}

std::shared_ptr<const MapSnapshot> MapReader::acquire(TimePoint now) const {
  std::lock_guard<std::mutex> lock(control_->mutex);
  return control_->Usable(control_->latest, now) ? control_->latest : nullptr;
}

std::shared_ptr<const MapSnapshot> MapReader::latestForDisplay() const {
  std::lock_guard<std::mutex> lock(control_->mutex);
  return control_->latest;
}

bool MapReader::isUsable(const std::shared_ptr<const MapSnapshot>& snapshot,
                         TimePoint now) const {
  std::lock_guard<std::mutex> lock(control_->mutex);
  return control_->Usable(snapshot, now);
}

ReaderStatus MapReader::status() const {
  std::lock_guard<std::mutex> lock(control_->mutex);
  return control_->status;
}

double EgoGridMapAdapter::getResolution() const {
  if (!snapshot_) {
    throw std::logic_error("no map snapshot");
  }
  return snapshot_->resolution();
}

int EgoGridMapAdapter::getInflateOccupancy(const Vec3& point) const {
  return !snapshot_ || snapshot_->isBlocked(point, policy_) ? 1 : 0;
}

struct OccupancyMapper::Impl {
  Config config;
  Index dimensions;
  Index inflation_steps;
  Vec3 origin = Vec3::Zero();
  std::size_t voxel_count = 0;

  std::vector<double> odds;
  std::vector<std::uint8_t> seen;
  std::vector<std::uint8_t> inflated;
  std::vector<std::uint8_t> states;
  std::vector<std::int32_t> votes;
  std::vector<std::uint32_t> marks;
  std::vector<std::size_t> touched;

  std::uint32_t frame_mark = 0;
  std::uint64_t epoch = 0;
  std::uint64_t sequence = 0;
  std::uint64_t last_frame = 0;
  double last_time = 0.0;
  bool have_frame = false;

  double hit_log = 0.0;
  double miss_log = 0.0;
  double min_log = 0.0;
  double max_log = 0.0;
  double free_log = 0.0;
  double occ_log = 0.0;

  struct Slot {
    std::shared_ptr<MapSnapshot> data;
    std::atomic<bool> busy{false};
  };

  std::array<std::shared_ptr<Slot>, 3> pool;
  std::shared_ptr<MapControl> control = std::make_shared<MapControl>();

  explicit Impl(const Config& cfg) : config(cfg) {
    const Layout layout = ValidatedLayout(config);
    dimensions = layout.dimensions;
    voxel_count = layout.voxel_count;
    inflation_steps = layout.inflation_steps;

    odds.resize(voxel_count);
    seen.resize(voxel_count);
    states.resize(voxel_count);
    inflated.resize(voxel_count);
    votes.resize(voxel_count);
    marks.resize(voxel_count);
    touched.reserve(voxel_count);

    for (auto& slot : pool) {
      slot = std::make_shared<Slot>();
      auto& snapshot = slot->data;
      snapshot = std::make_shared<MapSnapshot>();
      snapshot->dimensions_ = dimensions;
      snapshot->resolution_ = config.resolution;
      snapshot->states_.resize(voxel_count);
      snapshot->inflated_.resize(voxel_count);
    }

    hit_log = Logit(config.p_hit);
    miss_log = Logit(config.p_miss);
    min_log = Logit(config.p_min);
    max_log = Logit(config.p_max);
    free_log = Logit(config.p_free);
    occ_log = Logit(config.p_occ);
    control->max_age = config.max_age_sec;
  }

  void Vote(const Index& index, bool hit) {
    if (!InBounds(index, dimensions)) {
      return;
    }

    const auto offset = Address(index, dimensions);
    if (marks[offset] != frame_mark) {
      marks[offset] = frame_mark;
      votes[offset] = 0;
      touched.push_back(offset);
    }
    votes[offset] += hit ? 1 : -1;
  }

  bool Ray(const Vec3& camera, const Ray& ray) {
    if (!ray.endpoint.allFinite()) {
      return false;
    }

    const Vec3 direction_to_endpoint = ray.endpoint - camera;
    const double length = direction_to_endpoint.norm();
    if (!std::isfinite(length) || length <= config.min_ray_length) {
      return false;
    }

    bool hit = ray.hit && length <= config.max_ray_length;
    const Vec3 endpoint_world =
        camera + direction_to_endpoint * (std::min(length, config.max_ray_length) / length);
    const auto endpoint_index =
        IndexOf(endpoint_world, origin, dimensions, config.resolution);
    if (!endpoint_index) {
      hit = false;
    }
    if (hit) {
      Vote(*endpoint_index, true);
    }

    // Clip the ray to the grid and walk it cell by cell. The endpoint cell is
    // occupied; every other traversed cell receives a miss vote.
    const Vec3 start_grid =
        (camera + direction_to_endpoint * (config.min_ray_length / length) - origin) /
        config.resolution;
    const Vec3 end_grid = (endpoint_world - origin) / config.resolution;
    const Vec3 grid_direction = end_grid - start_grid;

    double enter = 0.0;
    double leave = 1.0;
    for (int axis = 0; axis < 3; ++axis) {
      if (grid_direction[axis] == 0) {
        if (start_grid[axis] < 0 || start_grid[axis] >= dimensions[axis]) {
          return hit;
        }
      } else {
        double near = (0 - start_grid[axis]) / grid_direction[axis];
        double far = (dimensions[axis] - start_grid[axis]) / grid_direction[axis];
        if (near > far) {
          std::swap(near, far);
        }
        enter = std::max(enter, near);
        leave = std::min(leave, far);
      }
    }
    if (enter > leave) {
      return hit;
    }

    Vec3 first = start_grid + enter * grid_direction;
    Vec3 last = start_grid + leave * grid_direction;
    for (int axis = 0; axis < 3; ++axis) {
      first[axis] = std::clamp(first[axis], 0.0,
                               std::nextafter(static_cast<double>(dimensions[axis]), 0.0));
      last[axis] = std::clamp(last[axis], 0.0,
                              std::nextafter(static_cast<double>(dimensions[axis]), 0.0));
    }

    Index index = first.array().floor().cast<int>();
    const Index finish = last.array().floor().cast<int>();
    const Vec3 walk = last - first;
    Vec3 t_max;
    Vec3 t_delta;
    Index step;
    for (int axis = 0; axis < 3; ++axis) {
      step[axis] = (walk[axis] > 0) - (walk[axis] < 0);
      t_delta[axis] = step[axis] == 0 ? std::numeric_limits<double>::infinity()
                                      : 1.0 / std::abs(walk[axis]);
      t_max[axis] = step[axis] > 0
                        ? (index[axis] + 1 - first[axis]) / walk[axis]
                        : step[axis] < 0 ? (index[axis] - first[axis]) / walk[axis]
                                         : std::numeric_limits<double>::infinity();
    }

    const std::size_t limit =
        static_cast<std::size_t>(dimensions.x()) + dimensions.y() + dimensions.z() + 3;
    for (std::size_t step_count = 0; step_count < limit; ++step_count) {
      if (!(hit && index == *endpoint_index)) {
        Vote(index, false);
      }
      if (index == finish) {
        break;
      }

      const double next = t_max.minCoeff();
      if (!std::isfinite(next)) {
        break;
      }
      for (int axis = 0; axis < 3; ++axis) {
        if (t_max[axis] <= next + 1e-12) {
          index[axis] += step[axis];
          t_max[axis] += t_delta[axis];
        }
      }
      if (!InBounds(index, dimensions)) {
        break;
      }
    }

    return true;
  }

  void ClassifyAndInflate() {
    std::fill(inflated.begin(), inflated.end(), 0);

    for (std::size_t offset = 0; offset < voxel_count; ++offset) {
      if (!seen[offset]) {
        states[offset] = static_cast<std::uint8_t>(VoxelState::Unknown);
      } else if (odds[offset] >= occ_log) {
        states[offset] = static_cast<std::uint8_t>(VoxelState::Occupied);
      } else if (odds[offset] <= free_log) {
        states[offset] = static_cast<std::uint8_t>(VoxelState::Free);
      } else {
        states[offset] = static_cast<std::uint8_t>(VoxelState::Unknown);
      }
    }

    for (int x = 0; x < dimensions.x(); ++x) {
      for (int y = 0; y < dimensions.y(); ++y) {
        for (int z = 0; z < dimensions.z(); ++z) {
          const Index index(x, y, z);
          if (states[Address(index, dimensions)] !=
              static_cast<std::uint8_t>(VoxelState::Occupied)) {
            continue;
          }

          const Index minimum = (index - inflation_steps).cwiseMax(Index::Zero());
          const Index maximum =
              (index + inflation_steps).cwiseMin(dimensions - Index::Ones());
          for (int xx = minimum.x(); xx <= maximum.x(); ++xx) {
            for (int yy = minimum.y(); yy <= maximum.y(); ++yy) {
              for (int zz = minimum.z(); zz <= maximum.z(); ++zz) {
                inflated[Address(Index(xx, yy, zz), dimensions)] = 1;
              }
            }
          }
        }
      }
    }
  }
};

OccupancyMapper::OccupancyMapper(const Config& config)
    : impl_(std::make_unique<Impl>(config)) {}

OccupancyMapper::~OccupancyMapper() = default;

MapReader OccupancyMapper::reader() const { return MapReader(impl_->control); }

const Config& OccupancyMapper::config() const { return impl_->config; }

void OccupancyMapper::reset(SourceStamp source, const Vec3& center) {
  auto& impl = *impl_;
  if (!center.allFinite()) {
    throw std::invalid_argument("nonfinite map center");
  }

  const Vec3 origin =
      center - impl.dimensions.cast<double>() * impl.config.resolution / 2.0;
  if (!origin.allFinite()) {
    throw std::invalid_argument("map origin overflow");
  }

  {
    std::lock_guard<std::mutex> lock(impl.control->mutex);
    impl.epoch = ++impl.control->status.epoch;
    impl.control->status.accepting = true;
    impl.control->status.valid = false;
    impl.control->status.reason = "waiting for observation";
    impl.control->source = source;
    impl.control->latest.reset();
  }

  impl.origin = origin;
  impl.have_frame = false;
  impl.sequence = 0;
  std::fill(impl.odds.begin(), impl.odds.end(), 0);
  std::fill(impl.seen.begin(), impl.seen.end(), 0);
  std::fill(impl.inflated.begin(), impl.inflated.end(), 0);
  std::fill(impl.states.begin(), impl.states.end(), 0);
}

void OccupancyMapper::invalidate(const std::string& reason) {
  auto& control = *impl_->control;
  std::lock_guard<std::mutex> lock(control.mutex);
  ++control.status.epoch;
  control.status.accepting = false;
  control.status.valid = false;
  control.status.reason = reason;
}

UpdateResult OccupancyMapper::integrate(const RayObservation& observation, TimePoint now) {
  auto& impl = *impl_;
  UpdateResult result;
  const TimePoint before = Clock::now();

  {
    std::lock_guard<std::mutex> lock(impl.control->mutex);
    if (!impl.control->status.accepting || impl.control->status.epoch != impl.epoch ||
        observation.source != impl.control->source) {
      result.reason = "source/epoch rejected";
      return result;
    }
  }

  if (!observation.origin.allFinite() || !std::isfinite(observation.timestamp_sec) ||
      observation.timestamp_sec < 0 ||
      !IndexOf(observation.origin, impl.origin, impl.dimensions, impl.config.resolution)) {
    result.reason = "invalid timestamp or camera outside map";
    return result;
  }

  if (now < observation.received ||
      std::chrono::duration<double>(now - observation.received).count() >
          impl.config.max_age_sec) {
    result.reason = "stale/future observation";
    return result;
  }

  if (impl.have_frame && (observation.frame_index <= impl.last_frame ||
                          observation.timestamp_sec <= impl.last_time)) {
    result.reason = "nonmonotonic observation";
    return result;
  }

  if (observation.rays.empty() || observation.rays.size() > impl.config.max_rays) {
    result.reason = "empty/oversized observation";
    return result;
  }

  impl.touched.clear();
  if (++impl.frame_mark == 0) {
    std::fill(impl.marks.begin(), impl.marks.end(), 0);
    impl.frame_mark = 1;
  }

  for (const auto& ray : observation.rays) {
    if (impl.Ray(observation.origin, ray)) {
      ++result.valid_rays;
    }
  }

  if (impl.touched.empty()) {
    result.reason = "no valid ray intersects map";
    return result;
  }

  for (const auto offset : impl.touched) {
    impl.seen[offset] = 1;
    impl.odds[offset] =
        std::clamp(impl.odds[offset] + (impl.votes[offset] >= 0 ? impl.hit_log : impl.miss_log),
                   impl.min_log, impl.max_log);
  }

  impl.have_frame = true;
  impl.last_frame = observation.frame_index;
  impl.last_time = observation.timestamp_sec;
  ++impl.sequence;

  const TimePoint fused = Clock::now();
  impl.ClassifyAndInflate();
  const TimePoint inflated = Clock::now();

  result.accepted = true;
  result.updated_voxels = impl.touched.size();
  result.fuse_ms = Milliseconds(before, fused);
  result.inflate_ms = Milliseconds(fused, inflated);

  std::shared_ptr<MapSnapshot> slot;
  for (auto& storage : impl.pool) {
    if (storage->busy.exchange(true, std::memory_order_acq_rel)) {
      continue;
    }

    // A fresh ownership block per publication makes old weak_ptrs expire, even
    // though the same bounded backing arrays are reused by a later lease.
    try {
      slot = std::shared_ptr<MapSnapshot>(
          storage->data.get(), [storage](MapSnapshot*) {
            storage->busy.store(false, std::memory_order_release);
          });
    } catch (...) {
      storage->busy.store(false, std::memory_order_release);
      throw;
    }
    break;
  }

  if (slot) {
    slot->origin_ = impl.origin;
    slot->states_ = impl.states;
    slot->inflated_ = impl.inflated;
    slot->metadata_.schema_version = 1;
    slot->metadata_.source = observation.source;
    slot->metadata_.epoch = impl.epoch;
    slot->metadata_.sequence = impl.sequence;
    slot->metadata_.frame_index = observation.frame_index;
    slot->metadata_.timestamp_sec = observation.timestamp_sec;
    slot->metadata_.received = observation.received;
    slot->metadata_.published = now;
    slot->metadata_.frame_id = impl.config.frame_id;

    std::lock_guard<std::mutex> lock(impl.control->mutex);
    if (impl.control->status.accepting && impl.control->status.epoch == impl.epoch) {
      impl.control->latest = slot;
      impl.control->status.valid = true;
      impl.control->status.reason = "ok";
      result.published = true;
    } else {
      result.reason = "invalidated during fusion";
    }
  } else {
    result.reason = "snapshot pool busy";
  }

  result.publish_ms = Milliseconds(inflated, Clock::now());
  return result;
}

}  // namespace ego_occupancy
