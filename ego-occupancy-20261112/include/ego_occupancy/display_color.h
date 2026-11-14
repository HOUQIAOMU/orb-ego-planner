#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <ego_occupancy/map.h>

namespace ego_occupancy {

enum class ColorMode { CameraDepth = 0, MapAxis = 1, Flat = 2 };
using Rgb8 = std::array<std::uint8_t, 3>;
inline constexpr Rgb8 kUnavailableColor{102, 102, 102};
inline constexpr Rgb8 kFlatColor{26, 204, 255};

struct ColorRange {
  double minimum_m = 0.1;
  double maximum_m = 4.5;

  bool valid() const;
};

/// Fixed red/yellow/green/cyan/blue lookup table; nonfinite input is gray.
Rgb8 rainbowColor(double fraction, bool reverse = false);

/// Reuses output storage. Camera mode measures optical Z, not Euclidean range.
/// Points behind the camera, invalid ranges/poses and unusable maps are gray.
/// MapAxis uses world -Y and does not imply gravity alignment or visibility.
void colorizePoints(const std::vector<Vec3>& points,
                    ColorMode mode,
                    ColorRange range,
                    bool reverse,
                    const std::optional<Eigen::Isometry3d>& T_world_camera,
                    bool map_usable,
                    std::vector<Rgb8>& colors);

}  // namespace ego_occupancy
