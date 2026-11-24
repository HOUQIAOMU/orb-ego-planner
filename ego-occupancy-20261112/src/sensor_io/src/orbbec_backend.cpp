#include "capture_backend.h"

#include <libobsensor/ObSensor.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ego_occupancy {
namespace {

std::shared_ptr<ob::Device> SelectDevice(ob::Context& context, const std::string& serial) {
  const auto devices = context.queryDeviceList();
  if (!devices || devices->getCount() == 0) throw std::runtime_error("no Orbbec camera found");
  if (!serial.empty()) return devices->getDeviceBySN(serial.c_str());
  if (devices->getCount() != 1)
    throw std::runtime_error("multiple Orbbec cameras found; specify --serial");
  return devices->getDevice(0);
}

std::shared_ptr<ob::VideoStreamProfile> FindProfile(const ob::StreamProfileList& list, int width,
                                                   int height, OBFormat format, int fps) {
  for (std::uint32_t i = 0; i < list.getCount(); ++i) {
    const auto profile = list.getProfile(i)->as<ob::VideoStreamProfile>();
    if (profile->getFormat() == format && profile->getFps() == static_cast<unsigned>(fps) &&
        (width == OB_WIDTH_ANY || profile->getWidth() == static_cast<unsigned>(width)) &&
        (height == OB_HEIGHT_ANY || profile->getHeight() == static_cast<unsigned>(height)))
      return profile;
  }
  return {};
}

cv::Matx33d IntrinsicMatrix(const OBCameraIntrinsic& intrinsics) {
  return {intrinsics.fx, 0, intrinsics.cx, 0, intrinsics.fy, intrinsics.cy, 0, 0, 1};
}

std::array<double, 8> DistortionCoefficients(const OBCameraDistortion& d) {
  return {d.k1, d.k2, d.p1, d.p2, d.k3, d.k4, d.k5, d.k6};
}

bool IsRectified(const OBCameraDistortion& distortion) {
  const auto values = DistortionCoefficients(distortion);
  return std::all_of(values.begin(), values.end(), [](double x) { return x == 0; });
}

bool MatchesCalibration(const OBCameraIntrinsic& intrinsics, const CameraModel& camera) {
  constexpr double tolerance = 0.001;
  return intrinsics.width == camera.width && intrinsics.height == camera.height &&
         std::abs(intrinsics.fx - camera.fx) <= tolerance &&
         std::abs(intrinsics.fy - camera.fy) <= tolerance &&
         std::abs(intrinsics.cx - camera.cx) <= tolerance &&
         std::abs(intrinsics.cy - camera.cy) <= tolerance;
}

cv::Mat PackedImage(const ob::VideoFrame& frame, int type) {
  const auto width = static_cast<int>(frame.getWidth());
  const auto height = static_cast<int>(frame.getHeight());
  const auto bytes = static_cast<std::size_t>(width) * height * CV_ELEM_SIZE(type);
  auto* data = frame.getData();
  if (width <= 0 || height <= 0 || !data || frame.getDataSize() != bytes)
    throw std::runtime_error("unexpected Orbbec image buffer size");
  return cv::Mat(height, width, type, data);
}

cv::Mat CopyBgr(const ob::ColorFrame& frame) {
  if (frame.getFormat() == OB_FORMAT_BGR) return PackedImage(frame, CV_8UC3).clone();
  cv::Mat bgr;
  switch (frame.getFormat()) {
    case OB_FORMAT_RGB:
      cv::cvtColor(PackedImage(frame, CV_8UC3), bgr, cv::COLOR_RGB2BGR);
      return bgr;
    case OB_FORMAT_YUYV:
      cv::cvtColor(PackedImage(frame, CV_8UC2), bgr, cv::COLOR_YUV2BGR_YUY2);
      return bgr;
    case OB_FORMAT_MJPG:
      if (!frame.getData() || frame.getDataSize() == 0)
        throw std::runtime_error("empty Orbbec MJPEG frame");
      return cv::imdecode(
          cv::Mat(1, static_cast<int>(frame.getDataSize()), CV_8UC1, frame.getData()),
          cv::IMREAD_COLOR);
    default:
      throw std::runtime_error("unsupported Orbbec color format");
  }
}

class OrbbecBackend final : public CaptureBackend {
 public:
  OrbbecBackend(const Options& options, const CameraModel& camera, int fps)
      : camera_(camera), fps_(fps), device_(SelectDevice(context_, options.serial)),
        pipeline_(device_) {
    const auto colors = pipeline_.getStreamProfileList(OB_SENSOR_COLOR);
    const auto depths = pipeline_.getStreamProfileList(OB_SENSOR_DEPTH);
    if (!colors || !depths) throw std::runtime_error("Orbbec camera requires color and depth streams");

    std::shared_ptr<ob::VideoStreamProfile> color;
    for (auto format : {OB_FORMAT_BGR, OB_FORMAT_RGB, OB_FORMAT_YUYV, OB_FORMAT_MJPG}) {
      color = FindProfile(*colors, camera.width, camera.height, format, fps);
      if (color) break;
    }
    auto depth = FindProfile(*depths, camera.width, camera.height, OB_FORMAT_Y16, fps);
    if (!depth) depth = FindProfile(*depths, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_Y16, fps);
    if (!color || !depth)
      throw std::runtime_error("no Orbbec color profile matching YAML and Y16 depth at the same fps");

    auto config = std::make_shared<ob::Config>();
    config->enableStream(color);
    config->enableStream(depth);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    // D2C targets the ideal color pinhole. Apply the corresponding correction to color below.
    align_.setConfigValue("TargetDistortion", 0);
    align_.setMatchTargetResolution(true);
    pipeline_.enableFrameSync();
    pipeline_.start(config);
  }

  void writeDeviceMetadata(cv::FileStorage& metadata) const override {
    const auto info = device_->getDeviceInfo();
    metadata << "serial" << info->getSerialNumber() << "firmware" << info->getFirmwareVersion()
             << "depth_scale_m_per_unit" << camera_.depth_scale << "fx" << camera_.fx << "fy"
             << camera_.fy << "cx" << camera_.cx << "cy" << camera_.cy << "fps" << fps_
             << "width" << camera_.width << "height" << camera_.height << "imu" << 0
             << "preprocessing" << "software D2C to rectified color";
  }

  std::optional<CaptureMetadata> read(int timeout_ms) override {
    frames_ = pipeline_.waitForFrameset(timeout_ms);
    if (!frames_) return std::nullopt;
    const auto received = Clock::now();
    const auto color = frames_->getColorFrame();
    const auto depth = frames_->getDepthFrame();
    if (!color || !depth) return std::nullopt;

    // Both timestamps come from the same device clock. CaptureMetadata keeps
    // its domain integers for compatibility with the common recorder.
    constexpr int device_clock = 0;
    return CaptureMetadata{color->getTimeStampUs() * 1e-6, depth->getTimeStampUs() * 1e-6,
                           color->getIndex(), depth->getIndex(), device_clock, device_clock,
                           received};
  }

  void copyImages(orb_live::RGBDFrame& output) override {
    const auto aligned = align_.process(frames_);
    if (!aligned || !aligned->is<ob::FrameSet>())
      throw std::runtime_error("Orbbec D2C did not return a frameset");
    const auto depth = aligned->as<ob::FrameSet>()->getDepthFrame();
    const auto color = frames_->getColorFrame();
    if (!depth || depth->getFormat() != OB_FORMAT_Y16 || !color)
      throw std::runtime_error("Orbbec D2C did not produce a Y16 depth frame");

    const cv::Size expected_size(camera_.width, camera_.height);
    const auto depth_profile = depth->getStreamProfile()->as<ob::VideoStreamProfile>();
    if (!MatchesCalibration(depth_profile->getIntrinsic(), camera_) ||
        !IsRectified(depth_profile->getDistortion()))
      throw std::runtime_error("Orbbec aligned depth profile differs from rectified camera YAML");

    const double scale = depth->getValueScale() * 0.001;  // SDK reports mm per raw unit.
    if (!std::isfinite(scale) || std::abs(scale / camera_.depth_scale - 1) > 1e-4)
      throw std::runtime_error("Orbbec depth units differ from RGBD.DepthMapFactor in camera YAML");

    const auto color_profile = color->getStreamProfile()->as<ob::VideoStreamProfile>();
    if (!MatchesCalibration(color_profile->getIntrinsic(), camera_))
      throw std::runtime_error("Orbbec color intrinsics differ from camera YAML");
    if (color_profile_ != color_profile) {
      PrepareColorRectification(*color_profile, expected_size);
      color_profile_ = color_profile;
    }

    cv::Mat bgr = CopyBgr(*color);
    const cv::Mat y16 = PackedImage(*depth, CV_16UC1);
    if (bgr.size() != expected_size || y16.size() != expected_size)
      throw std::runtime_error("Orbbec RGB-D output size differs from camera YAML");
    if (color_map_x_.empty())
      output.color = std::move(bgr);
    else
      cv::remap(bgr, output.color, color_map_x_, color_map_y_, cv::INTER_LINEAR);
    output.depth = y16.clone();
  }

  void stop() override { pipeline_.stop(); }

 private:
  void PrepareColorRectification(const ob::VideoStreamProfile& profile, const cv::Size& size) {
    const auto distortion = profile.getDistortion();
    if (IsRectified(distortion)) {
      color_map_x_.release();
      color_map_y_.release();
      return;
    }

    if (distortion.model != OB_DISTORTION_BROWN_CONRADY &&
        distortion.model != OB_DISTORTION_BROWN_CONRADY_K6)
      throw std::runtime_error("unsupported Orbbec color distortion model for rectification");
    auto values = DistortionCoefficients(distortion);
    if (!std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); }))
      throw std::runtime_error("invalid Orbbec color distortion coefficients");
    const auto intrinsic = IntrinsicMatrix(profile.getIntrinsic());
    const cv::Mat coefficients(1, static_cast<int>(values.size()), CV_64F, values.data());
    cv::initUndistortRectifyMap(intrinsic, coefficients, cv::noArray(), intrinsic, size,
                               CV_16SC2, color_map_x_, color_map_y_);
  }

  CameraModel camera_;
  int fps_;
  ob::Context context_;
  std::shared_ptr<ob::Device> device_;
  ob::Pipeline pipeline_;
  ob::Align align_{OB_STREAM_COLOR};
  std::shared_ptr<ob::FrameSet> frames_;
  std::shared_ptr<const ob::VideoStreamProfile> color_profile_;
  cv::Mat color_map_x_, color_map_y_;
};

}  // namespace

std::unique_ptr<CaptureBackend> openOrbbec(const Options& options, const CameraModel& camera,
                                          int fps) {
  return std::make_unique<OrbbecBackend>(options, camera, fps);
}

}  // namespace ego_occupancy
