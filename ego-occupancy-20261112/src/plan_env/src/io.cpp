#include <ego_occupancy/io.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace ego_occupancy {
namespace {

using Bytes = std::vector<std::uint8_t>;

void AppendUint(Bytes& bytes, std::uint64_t value, int width = 8) {
  for (int i = 0; i < width; ++i) {
    bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
  }
}

void AppendDouble(Bytes& bytes, double value) {
  static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
                "IEEE754 required");
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  AppendUint(bytes, bits);
}

std::uint64_t Hash(const Bytes& bytes, std::size_t length) {
  std::uint64_t hash = 14695981039346656037ULL;
  for (std::size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}

struct Cursor {
  const Bytes& bytes;
  std::size_t at = 0;

  std::uint64_t Get(int width = 8) {
    if (width < 0 || at + static_cast<std::size_t>(width) > bytes.size()) {
      throw std::runtime_error("truncated snapshot");
    }

    std::uint64_t value = 0;
    for (int i = 0; i < width; ++i) {
      value |= static_cast<std::uint64_t>(bytes[at++]) << (8 * i);
    }
    return value;
  }

  double Number() {
    const std::uint64_t bits = Get();
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    if (!std::isfinite(value)) {
      throw std::runtime_error("nonfinite snapshot geometry");
    }
    return value;
  }
};

void EnsureAbsent(const std::filesystem::path& path) {
  if (std::filesystem::exists(path)) {
    throw std::runtime_error("refusing to overwrite: " + path.string());
  }
}

}  // namespace

struct SnapshotIO {
  static void save(const MapSnapshot& snapshot, const std::filesystem::path& path) {
    EnsureAbsent(path);

    Bytes bytes;
    for (const char character : std::string("EGOOCC01")) {
      bytes.push_back(static_cast<std::uint8_t>(character));
    }
    AppendUint(bytes, 1, 4);
    for (int axis = 0; axis < 3; ++axis) {
      AppendUint(bytes, snapshot.dimensions_[axis], 4);
    }
    AppendDouble(bytes, snapshot.resolution_);
    for (int axis = 0; axis < 3; ++axis) {
      AppendDouble(bytes, snapshot.origin_[axis]);
    }

    const auto& metadata = snapshot.metadata_;
    AppendUint(bytes, metadata.source.session);
    AppendUint(bytes, metadata.source.map);
    AppendUint(bytes, metadata.source.revision);
    AppendUint(bytes, metadata.epoch);
    AppendUint(bytes, metadata.sequence);
    AppendUint(bytes, metadata.frame_index);
    AppendDouble(bytes, metadata.timestamp_sec);
    AppendUint(bytes, metadata.frame_id.size(), 4);
    for (const char character : metadata.frame_id) {
      bytes.push_back(static_cast<std::uint8_t>(character));
    }
    AppendUint(bytes, snapshot.states_.size());
    bytes.insert(bytes.end(), snapshot.states_.begin(), snapshot.states_.end());
    bytes.insert(bytes.end(), snapshot.inflated_.begin(), snapshot.inflated_.end());
    AppendUint(bytes, Hash(bytes, bytes.size()));

    auto temporary = path;
    temporary += ".partial";
    EnsureAbsent(temporary);

    try {
      std::ofstream output(temporary, std::ios::binary);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output.write(reinterpret_cast<const char*>(bytes.data()),
                   static_cast<std::streamsize>(bytes.size()));
      output.close();
      EnsureAbsent(path);
      std::filesystem::rename(temporary, path);
    } catch (...) {
      std::error_code error;
      std::filesystem::remove(temporary, error);
      throw;
    }
  }

  static std::shared_ptr<const MapSnapshot> load(const std::filesystem::path& path,
                                                 std::size_t maximum) {
    if (maximum == 0 || maximum > 100'000'000) {
      throw std::invalid_argument("invalid snapshot voxel limit");
    }

    // Header + one-character frame ID + one two-byte voxel + checksum.
    constexpr std::size_t kMinimumSize = 8 + 4 + 12 + 8 + 24 + 48 + 8 + 4 + 1 + 8 + 2 + 8;
    const auto size = std::filesystem::file_size(path);
    if (size < kMinimumSize || size > maximum * 2 + 2048) {
      throw std::runtime_error("snapshot length outside limit");
    }

    Bytes bytes(static_cast<std::size_t>(size));
    std::ifstream input(path, std::ios::binary);
    input.exceptions(std::ios::failbit | std::ios::badbit);
    input.read(reinterpret_cast<char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));

    Cursor checksum{bytes, bytes.size() - 8};
    if (checksum.Get() != Hash(bytes, bytes.size() - 8)) {
      throw std::runtime_error("snapshot checksum mismatch");
    }

    Cursor cursor{bytes, 0};
    for (const char character : std::string("EGOOCC01")) {
      if (cursor.Get(1) != static_cast<std::uint8_t>(character)) {
        throw std::runtime_error("invalid snapshot magic");
      }
    }
    if (cursor.Get(4) != 1) {
      throw std::runtime_error("unsupported snapshot version");
    }

    auto snapshot = std::make_shared<MapSnapshot>();
    std::size_t count = 1;
    for (int axis = 0; axis < 3; ++axis) {
      const auto dimension = cursor.Get(4);
      if (dimension == 0 ||
          dimension > static_cast<std::uint64_t>(std::numeric_limits<int>::max()) ||
          dimension > maximum / count) {
        throw std::runtime_error("snapshot dimensions exceed limit");
      }
      snapshot->dimensions_[axis] = static_cast<int>(dimension);
      count *= static_cast<std::size_t>(dimension);
    }

    snapshot->resolution_ = cursor.Number();
    if (snapshot->resolution_ <= 0) {
      throw std::runtime_error("invalid snapshot resolution");
    }
    for (int axis = 0; axis < 3; ++axis) {
      snapshot->origin_[axis] = cursor.Number();
    }
    if (!(snapshot->origin_ + snapshot->dimensions_.cast<double>() * snapshot->resolution_)
             .allFinite()) {
      throw std::runtime_error("snapshot bounds overflow");
    }

    auto& metadata = snapshot->metadata_;
    metadata.source = {cursor.Get(), cursor.Get(), cursor.Get()};
    metadata.epoch = cursor.Get();
    metadata.sequence = cursor.Get();
    metadata.frame_index = cursor.Get();
    metadata.timestamp_sec = cursor.Number();
    if (metadata.timestamp_sec < 0) {
      throw std::runtime_error("negative snapshot time");
    }

    const auto frame_id_length = cursor.Get(4);
    const auto frame_id_size = static_cast<std::size_t>(frame_id_length);
    if (frame_id_length == 0 || frame_id_length > 256 ||
        cursor.at + frame_id_size > bytes.size()) {
      throw std::runtime_error("invalid snapshot frame id");
    }
    metadata.frame_id.assign(bytes.begin() + cursor.at,
                             bytes.begin() + cursor.at + frame_id_size);
    cursor.at += frame_id_size;

    if (cursor.Get() != count || cursor.at + 2 * count + 8 != bytes.size()) {
      throw std::runtime_error("snapshot payload length mismatch");
    }

    snapshot->states_.assign(bytes.begin() + cursor.at,
                             bytes.begin() + cursor.at + count);
    cursor.at += count;
    snapshot->inflated_.assign(bytes.begin() + cursor.at,
                               bytes.begin() + cursor.at + count);
    for (std::size_t i = 0; i < count; ++i) {
      if (snapshot->states_[i] > static_cast<std::uint8_t>(VoxelState::Occupied) ||
          snapshot->inflated_[i] > 1) {
        throw std::runtime_error("invalid voxel state");
      }
    }

    // Steady-clock values are process-local and deliberately not deserialized.
    return snapshot;
  }
};

void saveSnapshot(const MapSnapshot& snapshot, const std::filesystem::path& path) {
  SnapshotIO::save(snapshot, path);
}

std::shared_ptr<const MapSnapshot> loadSnapshot(const std::filesystem::path& path,
                                                std::size_t max_voxels) {
  return SnapshotIO::load(path, max_voxels);
}

void exportPly(const MapSnapshot& snapshot,
               const std::filesystem::path& path,
               bool inflated) {
  EnsureAbsent(path);

  std::size_t count = 0;
  snapshot.forEach(Index::Zero(), snapshot.dimensions(),
                   [&count, inflated](const Index&, const Vec3&, VoxelQuery query) {
                     if (inflated ? query.inflated : query.state == VoxelState::Occupied) {
                       ++count;
                     }
                   });

  std::ofstream output(path);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output << "ply\nformat ascii 1.0\ncomment voxel centers in "
         << snapshot.metadata().frame_id << " meters\nelement vertex " << count
         << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
         << std::setprecision(9);

  snapshot.forEach(Index::Zero(), snapshot.dimensions(),
                   [&output, inflated](const Index&, const Vec3& point, VoxelQuery query) {
                     if (inflated ? query.inflated : query.state == VoxelState::Occupied) {
                       output << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
                     }
                   });
}

}  // namespace ego_occupancy
