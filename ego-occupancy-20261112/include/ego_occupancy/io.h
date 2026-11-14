#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>

#include <ego_occupancy/map.h>

namespace ego_occupancy {

void saveSnapshot(const MapSnapshot& snapshot, const std::filesystem::path& path);

std::shared_ptr<const MapSnapshot> loadSnapshot(const std::filesystem::path& path,
                                                std::size_t max_voxels = 8'000'000);

void exportPly(const MapSnapshot& snapshot,
               const std::filesystem::path& path,
               bool inflated = false);

}  // namespace ego_occupancy
