#include "capture_backend.h"
#include "orbbec_timing.h"

#include <libobsensor/ObSensor.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ego_occupancy {
namespace {

std::shared_ptr<ob::Device> SelectDevice(ob::Context& context, const std::string& serial) {
  if (ob::Version::getMajor() != 1) throw std::runtime_error("Orbbec SDK v1 is required");
  const auto devices = context.queryDeviceList();
  if (!devices || devices->deviceCount() == 0) throw std::runtime_error("no Orbbec camera found");
  if (!serial.empty()) return devices->getDeviceBySN(serial.c_str());
  if (devices->deviceCount() != 1)
    throw std::runtime_error("multiple Orbbec cameras found; specify --serial");
  return devices->getDevice(0);
}

std::shared_ptr<ob::VideoStreamProfile> FindProfile(ob::StreamProfileList& list, int width,
                                                   int height, OBFormat format, int fps) {
  for (std::uint32_t i = 0; i < list.count(); ++i) {
    const auto profile = list.getProfile(i)->as<ob::VideoStreamProfile>();
    if (profile->format() == format && profile->fps() == static_cast<unsigned>(fps) &&
        (width == OB_WIDTH_ANY || profile->width() == static_cast<unsigned>(width)) &&
        (height == OB_HEIGHT_ANY || profile->height() == static_cast<unsigned>(height)))
      return profile;
  }
  return {};
}

// These optional YAML fields specify the native depth stream, before D2C.
cv::Size NativeDepthSize(const Options& options) {
  const cv::FileStorage yaml(options.camera.string(), cv::FileStorage::READ);
  if (!yaml.isOpened()) throw std::runtime_error("cannot open Orbbec camera YAML");
  const auto width = yaml["Orbbec.DepthWidth"];
  const auto height = yaml["Orbbec.DepthHeight"];
  if (width.empty() && height.empty()) return {};
  if (!width.isInt() || !height.isInt() || static_cast<int>(width) <= 0 ||
      static_cast<int>(height) <= 0)
    throw std::runtime_error("Orbbec.DepthWidth and Orbbec.DepthHeight must both be positive integers");
  return {static_cast<int>(width), static_cast<int>(height)};
}

std::shared_ptr<ob::VideoStreamProfile> FindDepthProfile(
    ob::StreamProfileList& profiles, int width, int height, int fps) {
  for (const auto format : {OB_FORMAT_Y16, OB_FORMAT_Y11, OB_FORMAT_Z16, OB_FORMAT_Y12,
                            OB_FORMAT_Y14, OB_FORMAT_Y10}) {
    auto profile = FindProfile(profiles, width, height, format, fps);
    if (profile) return profile;
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

cv::Mat PackedImage(ob::VideoFrame& frame, int type) {
  const auto width = static_cast<int>(frame.width());
  const auto height = static_cast<int>(frame.height());
  const auto bytes = static_cast<std::size_t>(width) * height * CV_ELEM_SIZE(type);
  auto* data = frame.data();
  if (width <= 0 || height <= 0 || !data || frame.dataSize() != bytes)
    throw std::runtime_error("unexpected Orbbec image buffer size");
  return cv::Mat(height, width, type, data);
}

cv::Mat CopyBgr(ob::ColorFrame& frame) {
  if (frame.format() == OB_FORMAT_BGR) return PackedImage(frame, CV_8UC3).clone();
  cv::Mat bgr;
  switch (frame.format()) {
    case OB_FORMAT_RGB:
      cv::cvtColor(PackedImage(frame, CV_8UC3), bgr, cv::COLOR_RGB2BGR);
      return bgr;
    case OB_FORMAT_YUYV:
      cv::cvtColor(PackedImage(frame, CV_8UC2), bgr, cv::COLOR_YUV2BGR_YUY2);
      return bgr;
    case OB_FORMAT_MJPG:
      if (!frame.data() || frame.dataSize() == 0)
        throw std::runtime_error("empty Orbbec MJPEG frame");
      return cv::imdecode(
          cv::Mat(1, static_cast<int>(frame.dataSize()), CV_8UC1, frame.data()),
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
    const cv::Size native = NativeDepthSize(options);
    auto depth = native.empty() ? FindDepthProfile(*depths, camera.width, camera.height, fps)
                                : FindDepthProfile(*depths, native.width, native.height, fps);
    if (!depth && native.empty())
      depth = FindDepthProfile(*depths, OB_WIDTH_ANY, OB_HEIGHT_ANY, fps);
    if (!color || !depth)
      throw std::runtime_error("no Orbbec color/depth profiles matching YAML and requested fps");

    auto config = std::make_shared<ob::Config>();
    config->enableStream(color);
    config->enableStream(depth);
    config->setAlignMode(ALIGN_D2C_SW_MODE);
    config->setDepthScaleRequire(true);
    // The SV1301S_U3 does not support SDK FrameSync. Accept candidates here;
    // read() validates completeness, uniqueness and skew on the host clock.
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
    pipeline_.start(config);
    if (options.orbbec_timing) timing_ = std::make_unique<orb_live::OrbbecTimingLog>();
  }

  void writeDeviceMetadata(cv::FileStorage& metadata) const override {
    const auto info = device_->getDeviceInfo();
    metadata << "serial" << info->serialNumber() << "firmware" << info->firmwareVersion()
             << "sdk" << ob::Version::getVersion() << "timestamp_source" << "sdk_host_receive"
             << "depth_scale_m_per_unit" << camera_.depth_scale << "fx" << camera_.fx << "fy"
             << camera_.fy << "cx" << camera_.cx << "cy" << camera_.cy << "fps" << fps_
             << "width" << camera_.width << "height" << camera_.height << "imu" << 0
             << "preprocessing" << "software D2C to rectified color";
  }

  std::optional<CaptureMetadata> read(int timeout_ms) override {
    const auto frames = pipeline_.waitForFrames(timeout_ms);
    const auto received = Clock::now();
    if (!frames) {
      ++misses_.sdk_timeouts;
      RecordTiming(received, orb_live::TimingOutcome::SdkTimeout, nullptr, nullptr);
      return std::nullopt;
    }
    const auto color = frames->colorFrame();
    const auto depth = frames->depthFrame();
    if (!color || !depth) {
      ++misses_.incomplete;
      RecordTiming(received, orb_live::TimingOutcome::Incomplete, color, depth);
      return std::nullopt;
    }

    // Device timestamps for the two streams are not comparable on this camera.
    const auto color_us = color->systemTimeStampUs();
    const auto depth_us = depth->systemTimeStampUs();
    const auto color_index = color->index();
    const auto depth_index = depth->index();
    if (!pair_gate_.accept(color_index, color_us, depth_index, depth_us)) {
      const auto skew = color_us >= depth_us ? color_us - depth_us : depth_us - color_us;
      if (color_us && depth_us && skew > orb_live::kMaxRgbdPairSkewUs) {
        ++misses_.skew_rejected;
        if (misses_.skew_rejected == 1) misses_.min_skew_us = skew;
        misses_.min_skew_us = std::min(misses_.min_skew_us, skew);
        misses_.max_skew_us = std::max(misses_.max_skew_us, skew);
        RecordTiming(received, orb_live::TimingOutcome::SkewRejected, color, depth);
      } else {
        ++misses_.unusable;
        RecordTiming(received, orb_live::TimingOutcome::NoncurrentOrInvalid, color, depth);
      }
      return std::nullopt;
    }
    RecordTiming(received, orb_live::TimingOutcome::Accepted, color, depth);
    misses_ = {};
    if (!host_origin_us_) host_origin_us_ = std::min(color_us, depth_us);
    frames_ = frames;
    constexpr int host_receive_clock = 1;
    return CaptureMetadata{(color_us - *host_origin_us_) * 1e-6,
                           (depth_us - *host_origin_us_) * 1e-6,
                           color_index, depth_index, host_receive_clock, host_receive_clock,
                           received};
  }

  std::string frameTimeoutDetails() const override {
    std::ostringstream out;
    out << "Orbbec sdk_timeouts=" << misses_.sdk_timeouts
        << " incomplete=" << misses_.incomplete
        << " skew_rejected=" << misses_.skew_rejected
        << " noncurrent_or_invalid=" << misses_.unusable;
    if (misses_.skew_rejected) {
      out << " skew_us_min=" << misses_.min_skew_us << " skew_us_max=" << misses_.max_skew_us;
    }
    return out.str();
  }

  void copyImages(orb_live::RGBDFrame& output) override {
    const auto depth = frames_->depthFrame();
    const auto color = frames_->colorFrame();
    if (depth->format() != OB_FORMAT_Y16 && depth->format() != OB_FORMAT_Z16)
      throw std::runtime_error("Orbbec SDK v1 D2C must deliver unpacked Y16/Z16 depth");

    const double scale = depth->getValueScale() * 0.001;  // SDK reports mm per raw unit.
    if (!std::isfinite(scale) || std::abs(scale / camera_.depth_scale - 1) > 1e-4)
      throw std::runtime_error("Orbbec depth units differ from RGBD.DepthMapFactor in camera YAML");

    const cv::Size expected_size(camera_.width, camera_.height);
    if (!calibration_ready_) {
      // Query after the pipeline has produced a complete, aligned frame pair.
      const auto calibration = pipeline_.getCameraParam();
      if (!MatchesCalibration(calibration.rgbIntrinsic, camera_) ||
          !MatchesCalibration(calibration.depthIntrinsic, camera_) ||
          !IsRectified(calibration.depthDistortion))
        throw std::runtime_error("Orbbec SDK v1 D2C calibration differs from rectified camera YAML");
      PrepareColorRectification(calibration, expected_size);
      calibration_ready_ = true;
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

  void stop() override {
    pipeline_.stop();
    if (timing_) timing_->finish();
  }

 private:
  void RecordTiming(TimePoint received, orb_live::TimingOutcome outcome,
                    const std::shared_ptr<ob::Frame>& color,
                    const std::shared_ptr<ob::Frame>& depth) {
    if (!timing_) return;
    const auto timestamps = [](const std::shared_ptr<ob::Frame>& frame)
        -> std::optional<orb_live::FrameTiming> {
      if (!frame) return std::nullopt;
      return orb_live::FrameTiming{frame->index(), frame->timeStampUs(), frame->systemTimeStampUs()};
    };
    const auto stamp_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(received.time_since_epoch()).count();
    timing_->push({0, stamp_ns, outcome, timestamps(color), timestamps(depth)});
  }

  void PrepareColorRectification(const OBCameraParam& calibration, const cv::Size& size) {
    // The v1 calibration API exports Brown–Conrady coefficients without a model tag.
    if (IsRectified(calibration.rgbDistortion)) return;
    auto values = DistortionCoefficients(calibration.rgbDistortion);
    if (!std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); }))
      throw std::runtime_error("invalid Orbbec color distortion coefficients");
    const auto intrinsic = IntrinsicMatrix(calibration.rgbIntrinsic);
    const cv::Mat coefficients(1, static_cast<int>(values.size()), CV_64F, values.data());
    cv::initUndistortRectifyMap(intrinsic, coefficients, cv::noArray(), intrinsic, size,
                               CV_16SC2, color_map_x_, color_map_y_);
  }

  CameraModel camera_;
  int fps_;
  ob::Context context_;
  std::shared_ptr<ob::Device> device_;
  ob::Pipeline pipeline_;
  std::shared_ptr<ob::FrameSet> frames_;
  orb_live::HostPairGate pair_gate_;
  std::unique_ptr<orb_live::OrbbecTimingLog> timing_;
  struct {
    std::uint64_t sdk_timeouts = 0;
    std::uint64_t incomplete = 0;
    std::uint64_t skew_rejected = 0;
    std::uint64_t unusable = 0;
    std::uint64_t min_skew_us = 0;
    std::uint64_t max_skew_us = 0;
  } misses_;
  std::optional<std::uint64_t> host_origin_us_;
  bool calibration_ready_ = false;
  cv::Mat color_map_x_, color_map_y_;
};

}  // namespace

std::unique_ptr<CaptureBackend> openOrbbec(const Options& options, const CameraModel& camera,
                                          int fps) {
  return std::make_unique<OrbbecBackend>(options, camera, fps);
}

}  // namespace ego_occupancy
