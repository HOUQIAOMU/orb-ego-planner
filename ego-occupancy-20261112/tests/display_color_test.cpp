#include <ego_occupancy/display_color.h>

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using namespace ego_occupancy;

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool NearColor(const Rgb8& actual, const Rgb8& expected) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::abs(static_cast<int>(actual[i]) - expected[i]) > 3) {
      return false;
    }
  }
  return true;
}

void CameraDepthUsesTranslationAndOpticalZ() {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Vec3(10, 20, 30);
  const std::vector<Vec3> points{{10, 20, 31}, {110, 20, 31}, {10, 20, 35},
                                 {10, 20, 29}, {10, 20, 30}};
  std::vector<Rgb8> colors;
  colorizePoints(points, ColorMode::CameraDepth, {1, 5}, false, pose, true, colors);
  Require(colors.size() == points.size(), "one color per vertex required");
  Require(NearColor(colors[0], {255, 0, 0}), "near endpoint must be red");
  Require(colors[1] == colors[0], "optical Z must not become Euclidean range");
  Require(NearColor(colors[2], {0, 0, 255}), "far endpoint must be blue");
  Require(colors[3] == kUnavailableColor, "behind-camera point must be gray");
  Require(colors[4] == kUnavailableColor, "zero depth must be gray");

  pose.translation().z() = 28;
  colorizePoints({points[0]}, ColorMode::CameraDepth, {1, 5}, false, pose, true, colors);
  Require(NearColor(colors[0], {0, 255, 0}), "camera movement must recolor cached points");
}

void CameraRotationAndInvalidInputs() {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(std::acos(-1.0) / 2, Vec3::UnitY()).toRotationMatrix();
  std::vector<Rgb8> colors;
  colorizePoints({Vec3(3, 0, 100)}, ColorMode::CameraDepth, {1, 5}, false,
                 pose, true, colors);
  Require(NearColor(colors[0], {0, 255, 0}), "rotation must use camera forward axis");
  colorizePoints({Vec3(0, 0, 1)}, ColorMode::CameraDepth, {1, 5}, false,
                 std::nullopt, true, colors);
  Require(colors[0] == kUnavailableColor, "missing pose must not retain depth color");
  colorizePoints({Vec3(0, 0, 1)}, ColorMode::CameraDepth, {1, 5}, false,
                 Eigen::Isometry3d::Identity(), false, colors);
  Require(colors[0] == kUnavailableColor, "invalid map must be gray");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  colorizePoints({Vec3(nan, 0, 1)}, ColorMode::CameraDepth, {1, 5}, false,
                 Eigen::Isometry3d::Identity(), true, colors);
  Require(colors[0] == kUnavailableColor, "nonfinite point must not index palette");
  pose.translation().x() = nan;
  colorizePoints({Vec3(0, 0, 1)}, ColorMode::CameraDepth, {1, 5}, false,
                 pose, true, colors);
  Require(colors[0] == kUnavailableColor, "nonfinite camera must be gray");
}

void RangeReverseAndFallbackModes() {
  std::vector<Rgb8> colors;
  const auto pose = Eigen::Isometry3d::Identity();
  colorizePoints({Vec3(0, 0, 0.5), Vec3(0, 0, 8)}, ColorMode::CameraDepth,
                 {1, 5}, true, pose, true, colors);
  Require(NearColor(colors[0], {0, 0, 255}), "reversed near clipping incorrect");
  Require(NearColor(colors[1], {255, 0, 0}), "reversed far clipping incorrect");
  for (const ColorRange range : {ColorRange{1, 1}, ColorRange{5, 1},
                                 ColorRange{0, std::numeric_limits<double>::infinity()}}) {
    colorizePoints({Vec3(0, 0, 3)}, ColorMode::CameraDepth, range, false,
                   pose, true, colors);
    Require(colors[0] == kUnavailableColor, "invalid range must gray without throwing");
  }
  colorizePoints({Vec3(99, -1, 42), Vec3(99, -5, 42)}, ColorMode::MapAxis,
                 {1, 5}, false, std::nullopt, true, colors);
  Require(NearColor(colors[0], {255, 0, 0}), "map axis must use world -Y");
  Require(NearColor(colors[1], {0, 0, 255}), "axis mode must not require camera pose");
  colorizePoints({Vec3(0, 0, 3)}, ColorMode::Flat, {1, 5}, false,
                 std::nullopt, true, colors);
  Require(colors[0] == kFlatColor, "flat mode must remain available without pose");
  colorizePoints({Vec3(0, 0, 3)}, ColorMode::Flat, {1, 5}, false,
                 pose, false, colors);
  Require(colors[0] == kUnavailableColor, "flat mode cannot hide map invalidity");
  colorizePoints({}, ColorMode::CameraDepth, {1, 5}, false, pose, true, colors);
  Require(colors.empty(), "empty map must clear prior colors");
}
}  // namespace

int main() {
  try {
    CameraDepthUsesTranslationAndOpticalZ();
    CameraRotationAndInvalidInputs();
    RangeReverseAndFallbackModes();
    std::cout << "camera depth, palette and invalidation contracts passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
