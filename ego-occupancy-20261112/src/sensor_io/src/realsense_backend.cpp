#include "capture_backend.h"

#include <librealsense2/rs.hpp>

#include <cmath>
#include <stdexcept>

namespace ego_occupancy {
namespace {

class RealSenseBackend final : public CaptureBackend {
 public:
  RealSenseBackend(const Options& options, const CameraModel& model, int fps)
      : camera(model), serial_(options.serial), fps_(fps) {
    try {
      rs2::context context;
      bool found = false;
      for (auto device : context.query_devices()) {
        if (!device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ||
            options.serial != device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
          continue;
        }

        auto sensor = device.first<rs2::depth_sensor>();
        if (!sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
          throw std::runtime_error("emitter control unsupported");
        }
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
        found = true;
      }
      if (!found) {
        throw std::runtime_error("D435i not found: check serial, USB and camera ownership");
      }

      rs2::config config;
      config.enable_device(options.serial);
      config.enable_stream(RS2_STREAM_COLOR, camera.width, camera.height, RS2_FORMAT_BGR8,
                           fps);
      config.enable_stream(RS2_STREAM_DEPTH, camera.width, camera.height, RS2_FORMAT_Z16,
                           fps);

      auto profile = pipeline_.start(config);
      streaming_ = true;
      device_ = profile.get_device();
      auto color = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      auto depth = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
      intrinsics_ = color.get_intrinsics();
      const auto& intrinsics = intrinsics_;

      if (color.width() != camera.width || color.height() != camera.height ||
          color.fps() != fps || color.format() != RS2_FORMAT_BGR8 ||
          depth.width() != camera.width || depth.height() != camera.height ||
          depth.fps() != fps || depth.format() != RS2_FORMAT_Z16) {
        throw std::runtime_error("actual stream profile differs from camera YAML");
      }
      if (std::abs(intrinsics.fx - camera.fx) > 0.001 ||
          std::abs(intrinsics.fy - camera.fy) > 0.001 ||
          std::abs(intrinsics.ppx - camera.cx) > 0.001 ||
          std::abs(intrinsics.ppy - camera.cy) > 0.001) {
        throw std::runtime_error("active intrinsics differ from YAML");
      }
      for (const float coefficient : intrinsics.coeffs) {
        if (coefficient != 0) {
          throw std::runtime_error("nonzero live color distortion requires verified "
                                   "rectification");
        }
      }

      auto sensor = device_.first<rs2::depth_sensor>();
      depth_scale_ = sensor.get_depth_scale();
      if (std::abs(depth_scale_ / camera.depth_scale - 1) > 1e-4 ||
          sensor.get_option(RS2_OPTION_EMITTER_ENABLED) != 0) {
        throw std::runtime_error("depth units/emitter differ from baseline");
      }
    } catch (...) {
      try {
        stop();
      } catch (...) {
        // Keep the original startup error.
      }
      throw;
    }
  }

  void writeDeviceMetadata(cv::FileStorage& metadata) const override {
    metadata << "serial" << serial_ << "firmware"
             << device_.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "sdk_api"
             << RS2_API_VERSION_STR << "depth_scale_m_per_unit" << depth_scale_ << "fx"
             << static_cast<double>(intrinsics_.fx) << "fy"
             << static_cast<double>(intrinsics_.fy) << "cx"
             << static_cast<double>(intrinsics_.ppx) << "cy"
             << static_cast<double>(intrinsics_.ppy) << "fps" << fps_ << "width"
             << camera.width << "height" << camera.height << "emitter" << 0 << "imu"
             << 0 << "preprocessing" << "depth aligned to zero-distortion color";
    if (device_.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
      metadata << "usb_type"
               << device_.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
    }
  }

  std::optional<CaptureMetadata> read(int timeout_ms) override {
    if (!pipeline_.try_wait_for_frames(&frames_, timeout_ms)) return std::nullopt;
    const auto received = Clock::now();
    color_ = frames_.get_color_frame();
    auto depth = frames_.get_depth_frame();
    if (!color_ || !depth) throw std::runtime_error("incomplete RGB-D frameset");

    return CaptureMetadata{color_.get_timestamp() * 0.001, depth.get_timestamp() * 0.001,
                           color_.get_frame_number(), depth.get_frame_number(),
                           color_.get_frame_timestamp_domain(), depth.get_frame_timestamp_domain(),
                           received};
  }

  void copyImages(orb_live::RGBDFrame& output) override {
    auto aligned =
        align_.process(frames_).as<rs2::frameset>().get_depth_frame();
    auto aligned_intrinsics =
        aligned.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    if (aligned.get_width() != camera.width ||
        aligned.get_height() != camera.height ||
        std::abs(aligned_intrinsics.fx - camera.fx) > 0.001 ||
        std::abs(aligned_intrinsics.fy - camera.fy) > 0.001 ||
        std::abs(aligned_intrinsics.ppx - camera.cx) > 0.001 ||
        std::abs(aligned_intrinsics.ppy - camera.cy) > 0.001) {
      throw std::runtime_error("aligned depth calibration differs from color model");
    }

    output.color = cv::Mat(camera.height, camera.width, CV_8UC3,
                           const_cast<void*>(color_.get_data()),
                           color_.get_stride_in_bytes())
                       .clone();
    output.depth = cv::Mat(camera.height, camera.width, CV_16UC1,
                           const_cast<void*>(aligned.get_data()),
                           aligned.get_stride_in_bytes())
                       .clone();
  }

  void stop() override {
    if (streaming_) {
      streaming_ = false;
      pipeline_.stop();
    }
  }

 private:
  CameraModel camera;
  std::string serial_;
  int fps_;
  rs2::pipeline pipeline_;
  rs2::device device_;
  rs2_intrinsics intrinsics_{};
  double depth_scale_ = 0;
  bool streaming_ = false;
  rs2::align align_{RS2_STREAM_COLOR};
  rs2::frameset frames_;
  rs2::video_frame color_{rs2::frame{}};
};

}  // namespace

std::unique_ptr<CaptureBackend> openRealSense(const Options& options, const CameraModel& camera,
                                             int fps) {
  return std::make_unique<RealSenseBackend>(options, camera, fps);
}

}  // namespace ego_occupancy
