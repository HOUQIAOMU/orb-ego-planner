#include <ego_occupancy/depth.h>

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace ego_occupancy {
namespace {

double Number(const cv::FileStorage& storage, const char* name) {
  const auto node = storage[name];
  if (node.empty() || (!node.isInt() && !node.isReal())) {
    throw std::invalid_argument(std::string("missing numeric camera/config key: ") + name);
  }

  const double value = static_cast<double>(node);
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string("nonfinite key: ") + name);
  }
  return value;
}

}  // namespace

DepthProjector::DepthProjector(const CameraModel& camera) : camera_(camera) {
  if (camera.width <= 0 || camera.height <= 0 ||
      static_cast<std::int64_t>(camera.width) * camera.height > 16'000'000 ||
      camera.stride <= 0 || !std::isfinite(camera.fx) || !std::isfinite(camera.fy) ||
      camera.fx <= 0 || camera.fy <= 0 || !std::isfinite(camera.cx) ||
      !std::isfinite(camera.cy) || !std::isfinite(camera.depth_scale) ||
      camera.depth_scale <= 0 || !std::isfinite(camera.min_depth) ||
      !std::isfinite(camera.max_depth) || camera.min_depth <= 0 ||
      camera.max_depth <= camera.min_depth) {
    throw std::invalid_argument("invalid depth camera model");
  }

  for (const double coefficient : camera.distortion) {
    if (!std::isfinite(coefficient)) {
      throw std::invalid_argument("nonfinite distortion");
    }
  }
  if (camera.distortion_model != "rectified" &&
      camera.distortion_model != "brown_conrady") {
    throw std::invalid_argument("unsupported depth projection model");
  }
  if (camera.distortion_model == "rectified") {
    for (const double coefficient : camera.distortion) {
      if (coefficient != 0.0) {
        throw std::invalid_argument("rectified model with nonzero distortion");
      }
    }
  }

  std::vector<cv::Point2d> pixels;
  pixels.reserve(static_cast<std::size_t>(camera.width) * camera.height);
  for (int v = 0; v < camera.height; v += camera.stride) {
    for (int u = 0; u < camera.width; u += camera.stride) {
      pixels.emplace_back(u, v);
    }
  }

  std::vector<cv::Point2d> normalized;
  if (camera.distortion_model == "brown_conrady") {
    const cv::Matx33d intrinsics(camera.fx, 0, camera.cx, 0, camera.fy, camera.cy, 0, 0, 1);
    const cv::Mat distortion(1, 5, CV_64F,
                             const_cast<double*>(camera.distortion.data()));
    cv::undistortPoints(pixels, normalized, intrinsics, distortion);
  } else {
    for (const auto& pixel : pixels) {
      normalized.emplace_back((pixel.x - camera.cx) / camera.fx,
                              (pixel.y - camera.cy) / camera.fy);
    }
  }

  for (const auto& pixel : normalized) {
    const Vec3 ray(pixel.x, pixel.y, 1.0);
    if (!ray.allFinite()) {
      throw std::invalid_argument("invalid calibrated ray");
    }
    rays_.push_back(ray);
  }
}

RayObservation DepthProjector::project(const cv::Mat& depth,
                                       const Eigen::Isometry3d& pose,
                                       SourceStamp source,
                                       std::uint64_t frame_index,
                                       double timestamp_sec,
                                       TimePoint received) const {
  if (depth.cols != camera_.width || depth.rows != camera_.height ||
      (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)) {
    throw std::invalid_argument("depth size/encoding differs from calibration");
  }

  if (!pose.matrix().allFinite() || !std::isfinite(timestamp_sec) || timestamp_sec < 0 ||
      (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() >
          1e-5 ||
      std::abs(pose.linear().determinant() - 1.0) > 1e-5 ||
      (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-8) {
    throw std::invalid_argument("invalid rigid pose or timestamp");
  }

  RayObservation observation;
  observation.source = source;
  observation.frame_index = frame_index;
  observation.timestamp_sec = timestamp_sec;
  observation.received = received;
  observation.origin = pose.translation();
  observation.rays.reserve(rays_.size());

  std::size_t ray_index = 0;
  for (int v = 0; v < camera_.height; v += camera_.stride) {
    for (int u = 0; u < camera_.width; u += camera_.stride, ++ray_index) {
      const double z = depth.type() == CV_16UC1
                           ? depth.at<std::uint16_t>(v, u) * camera_.depth_scale
                           : static_cast<double>(depth.at<float>(v, u));
      if (!std::isfinite(z) || z < camera_.min_depth || z > camera_.max_depth) {
        continue;
      }

      const Vec3 point = pose * (rays_[ray_index] * z);
      if (point.allFinite()) {
        observation.rays.push_back({point, true});
      }
    }
  }

  return observation;
}

CameraModel readCameraModel(const std::string& path, int stride) {
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::invalid_argument("cannot open camera YAML: " + path);
  }

  if (!storage["File.version"].isString()) {
    throw std::invalid_argument("camera YAML requires string File.version=1.0");
  }
  std::string version;
  storage["File.version"] >> version;
  if (version != "1.0") {
    throw std::invalid_argument("unsupported camera File.version");
  }

  std::string type;
  storage["Camera.type"] >> type;
  if (type != "Rectified") {
    throw std::invalid_argument("first ORB entry requires verified Rectified camera YAML");
  }

  for (const char* key : {"Camera.newWidth", "Camera.newHeight", "System.LoadAtlasFromFile",
                          "System.SaveAtlasToFile"}) {
    if (!storage[key].empty()) {
      throw std::invalid_argument(std::string("unsupported camera key: ") + key);
    }
  }

  if (Number(storage, "Camera.RGB") != 0) {
    throw std::invalid_argument("expected BGR input (Camera.RGB=0)");
  }

  CameraModel camera;
  const double width = Number(storage, "Camera.width");
  const double height = Number(storage, "Camera.height");
  if (width < 1 || height < 1 || width > 10000 || height > 10000 ||
      std::floor(width) != width || std::floor(height) != height) {
    throw std::invalid_argument("invalid camera dimensions");
  }

  camera.width = static_cast<int>(width);
  camera.height = static_cast<int>(height);
  camera.fx = Number(storage, "Camera1.fx");
  camera.fy = Number(storage, "Camera1.fy");
  camera.cx = Number(storage, "Camera1.cx");
  camera.cy = Number(storage, "Camera1.cy");
  camera.depth_scale = 1.0 / Number(storage, "RGBD.DepthMapFactor");
  camera.stride = stride;

  // Constructing a projector performs the full model check.
  const DepthProjector check(camera);
  (void)check;
  return camera;
}

Config readMapConfig(const std::string& path) {
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::invalid_argument("cannot open map YAML: " + path);
  }

  const auto readVector = [&storage](const char* name) {
    const auto node = storage[name];
    if (!node.isSeq() || node.size() != 3) {
      throw std::invalid_argument(std::string("expected 3-vector: ") + name);
    }

    Vec3 value;
    for (int i = 0; i < 3; ++i) {
      if (!node[i].isInt() && !node[i].isReal()) {
        throw std::invalid_argument("nonnumeric vector");
      }
      value[i] = static_cast<double>(node[i]);
    }
    return value;
  };

  Config config;
  config.size = readVector("size_m");
  config.inflation = readVector("inflation_m");
  config.resolution = Number(storage, "resolution_m");
  config.min_ray_length = Number(storage, "min_ray_m");
  config.max_ray_length = Number(storage, "max_ray_m");
  config.max_age_sec = Number(storage, "max_age_sec");
  config.p_hit = Number(storage, "p_hit");
  config.p_miss = Number(storage, "p_miss");
  config.p_min = Number(storage, "p_min");
  config.p_max = Number(storage, "p_max");
  config.p_free = Number(storage, "p_free");
  config.p_occ = Number(storage, "p_occ");
  storage["frame_id"] >> config.frame_id;
  return config;
}

}  // namespace ego_occupancy
