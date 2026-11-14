#include <ego_occupancy/current_frame.h>

#include <cmath>
#include <stdexcept>

namespace ego_occupancy {

CurrentFrameBuffer::CurrentFrameBuffer(const CameraModel& camera)
    : width_(camera.width), height_(camera.height) {
  if (width_ <= 0 || height_ <= 0) {
    throw std::invalid_argument("current-frame dimensions must be positive");
  }
}

void CurrentFrameBuffer::publish(const cv::Mat& bgr,
                                 const ego_pose_bridge::Record& pose,
                                 std::uint64_t capture_index,
                                 TimePoint received,
                                 bool verified) {
  if (bgr.type() != CV_8UC3 || bgr.cols != width_ || bgr.rows != height_) {
    throw std::invalid_argument("current-frame image differs from calibrated BGR8 dimensions");
  }

  auto frame = std::make_shared<CurrentFrame>();
  frame->bgr = bgr.clone();
  frame->pose = pose;
  frame->capture_index = capture_index;
  frame->received = received;
  frame->pose_verified = verified;

  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(frame);
}

std::shared_ptr<const CurrentFrame> CurrentFrameBuffer::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

CurrentFrameSample CurrentFrameBuffer::sample() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {latest_, Clock::now()};
}

std::optional<Eigen::Isometry3d> cameraPoseForMap(const CurrentFrame& frame,
                                                  const MapSnapshot& map,
                                                  const ReaderStatus& status,
                                                  TimePoint now,
                                                  double max_age_sec) {
  const auto& record = frame.pose;
  const double age = std::chrono::duration<double>(now - frame.received).count();
  const SourceStamp source{record.session, record.map_token, record.revision};

  if (!frame.pose_verified || !record.captured || !ego_pose_bridge::IsCurrent(record) ||
      !status.accepting || !status.valid || status.epoch != map.metadata().epoch ||
      source != map.metadata().source || !std::isfinite(max_age_sec) || max_age_sec <= 0 ||
      age < 0 || age > max_age_sec) {
    return std::nullopt;
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.matrix() = Eigen::Map<const Eigen::Matrix4f>(record.T_world_camera.data()).cast<double>();
  if (!pose.matrix().allFinite() ||
      !pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 0.001) ||
      !(pose.linear().transpose() * pose.linear())
           .isApprox(Eigen::Matrix3d::Identity(), 0.001) ||
      pose.linear().determinant() < 0.99) {
    return std::nullopt;
  }

  return pose;
}

std::array<Vec3, 5> cameraFrustum(const Eigen::Isometry3d& pose,
                                  const CameraModel& camera,
                                  double depth) {
  if (camera.width <= 0 || camera.height <= 0 || !std::isfinite(camera.fx) ||
      !std::isfinite(camera.fy) || camera.fx <= 0 || camera.fy <= 0 ||
      !std::isfinite(camera.cx) || !std::isfinite(camera.cy) || !std::isfinite(depth) ||
      depth <= 0) {
    throw std::invalid_argument("invalid camera frustum calibration");
  }

  std::array<Vec3, 5> points;
  points[0] = pose.translation();

  const double pixels[4][2] = {{0.0, 0.0},
                               {static_cast<double>(camera.width - 1), 0.0},
                               {static_cast<double>(camera.width - 1),
                                static_cast<double>(camera.height - 1)},
                               {0.0, static_cast<double>(camera.height - 1)}};
  for (int i = 0; i < 4; ++i) {
    points[i + 1] = pose * Vec3((pixels[i][0] - camera.cx) * depth / camera.fx,
                                (pixels[i][1] - camera.cy) * depth / camera.fy,
                                depth);
  }
  return points;
}

}  // namespace ego_occupancy
