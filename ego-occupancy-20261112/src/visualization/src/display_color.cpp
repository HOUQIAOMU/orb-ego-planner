#include <ego_occupancy/display_color.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace ego_occupancy {
namespace {

const std::array<Rgb8, 256>& RainbowTable() {
  static const auto table = [] {
    constexpr std::array<Rgb8, 5> anchors{{{255, 0, 0}, {255, 255, 0},
                                         {0, 255, 0}, {0, 255, 255}, {0, 0, 255}}};
    std::array<Rgb8, 256> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
      const double position = static_cast<double>(i) * 4.0 / 255.0;
      const auto segment = std::min<std::size_t>(static_cast<std::size_t>(position), 3);
      const double weight = position - static_cast<double>(segment);
      for (std::size_t channel = 0; channel < 3; ++channel) {
        const double value = (1 - weight) * anchors[segment][channel] +
                             weight * anchors[segment + 1][channel];
        result[i][channel] = static_cast<std::uint8_t>(std::lround(value));
      }
    }
    return result;
  }();
  return table;
}

}  // namespace

bool ColorRange::valid() const {
  return std::isfinite(minimum_m) && std::isfinite(maximum_m) && maximum_m > minimum_m &&
         std::isfinite(maximum_m - minimum_m);
}

Rgb8 rainbowColor(double fraction, bool reverse) {
  if (!std::isfinite(fraction)) {
    return kUnavailableColor;
  }
  auto index = static_cast<std::size_t>(std::lround(std::clamp(fraction, 0.0, 1.0) * 255));
  if (reverse) {
    index = 255 - index;
  }
  return RainbowTable()[index];
}

void colorizePoints(const std::vector<Vec3>& points,
                    ColorMode mode,
                    ColorRange range,
                    bool reverse,
                    const std::optional<Eigen::Isometry3d>& T_world_camera,
                    bool map_usable,
                    std::vector<Rgb8>& colors) {
  colors.resize(points.size());
  std::fill(colors.begin(), colors.end(), kUnavailableColor);
  if (!map_usable) {
    return;
  }
  if (mode == ColorMode::Flat) {
    std::fill(colors.begin(), colors.end(), kFlatColor);
    return;
  }
  if (!range.valid()) {
    return;
  }

  Vec3 forward = Vec3::Zero();
  double offset = 0;
  if (mode == ColorMode::CameraDepth) {
    if (!T_world_camera || !T_world_camera->matrix().allFinite() || range.minimum_m < 0) {
      return;
    }
    // The third row of T_camera_world is enough: one dot product per point.
    forward = T_world_camera->linear().col(2);
    offset = -forward.dot(T_world_camera->translation());
  } else if (mode != ColorMode::MapAxis) {
    return;
  }

  const double inverse_span = 1.0 / (range.maximum_m - range.minimum_m);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& point = points[i];
    if (!point.allFinite()) {
      continue;
    }
    const double value = mode == ColorMode::CameraDepth ? forward.dot(point) + offset
                                                       : -point.y();
    if (!std::isfinite(value) || (mode == ColorMode::CameraDepth && value <= 0)) {
      continue;
    }
    colors[i] = rainbowColor((value - range.minimum_m) * inverse_span, reverse);
  }
}

}  // namespace ego_occupancy
