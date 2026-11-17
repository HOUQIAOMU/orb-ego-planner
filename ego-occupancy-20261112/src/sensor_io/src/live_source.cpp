#include "live_source.h"

#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace ego_occupancy {

struct LiveSource::Impl {
  Options options;
  CameraModel camera;
  int fps = 0;
  rs2::pipeline pipeline;
  orb_live::BoundedQueue<orb_live::FramePtr> live{2};
  orb_live::BoundedQueue<orb_live::FramePtr> disk{64, true};
  orb_live::SequenceWriter writer;
  std::atomic<bool> stopping{false};
  std::atomic<std::uint64_t> captured{0};
  std::atomic<std::uint64_t> recorded{0};
  std::atomic<std::uint64_t> rejected{0};
  std::atomic<std::uint64_t> color_gaps{0};
  std::atomic<std::uint64_t> depth_gaps{0};
  std::thread capture_thread;
  std::thread record_thread;
  std::string capture_error;
  std::string record_error;
  bool streaming = false;
  bool stopped = false;

  Impl(const Options& source_options, const CameraModel& camera_model, int frame_rate)
      : options(source_options),
        camera(camera_model),
        fps(frame_rate),
        writer(options.output / "rgbd") {
    try {
      std::filesystem::copy_file(options.camera, options.output / "rgbd/D435i.yaml");

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

      auto profile = pipeline.start(config);
      streaming = true;
      auto device = profile.get_device();
      auto color = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
      auto depth = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
      auto intrinsics = color.get_intrinsics();

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

      auto sensor = device.first<rs2::depth_sensor>();
      const double depth_scale = sensor.get_depth_scale();
      if (std::abs(depth_scale / camera.depth_scale - 1) > 1e-4 ||
          sensor.get_option(RS2_OPTION_EMITTER_ENABLED) != 0) {
        throw std::runtime_error("depth units/emitter differ from baseline");
      }

      cv::FileStorage metadata((options.output / "rgbd/device.yml").string(),
                               cv::FileStorage::WRITE);
      if (!metadata.isOpened()) {
        throw std::runtime_error("cannot write camera metadata");
      }
      metadata << "serial" << options.serial << "firmware"
               << device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "sdk_api"
               << RS2_API_VERSION_STR << "depth_scale_m_per_unit" << depth_scale << "fx"
               << static_cast<double>(intrinsics.fx) << "fy"
               << static_cast<double>(intrinsics.fy) << "cx"
               << static_cast<double>(intrinsics.ppx) << "cy"
               << static_cast<double>(intrinsics.ppy) << "fps" << fps << "width"
               << camera.width << "height" << camera.height << "emitter" << 0 << "imu"
               << 0 << "preprocessing" << "depth aligned to zero-distortion color";
      if (device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
        metadata << "usb_type"
                 << device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
      }

      record_thread = std::thread([this] { RecordLoop(); });
      capture_thread = std::thread([this] { CaptureLoop(); });
    } catch (...) {
      Stop();
      throw;
    }
  }

  void CaptureLoop() {
    try {
      rs2::align align(RS2_STREAM_COLOR);
      orb_live::TimestampGuard timestamps(0.010);
      int warmup = options.warmup;
      std::uint64_t previous_color = 0;
      std::uint64_t previous_depth = 0;
      auto last_frame = Clock::now();
      unsigned consecutive_bad = 0;

      while (!stopping) {
        rs2::frameset frames;
        if (!pipeline.try_wait_for_frames(&frames, 200)) {
          if (std::chrono::duration<double>(Clock::now() - last_frame).count() > 5) {
            throw std::runtime_error("no RGB-D frames for 5 seconds");
          }
          continue;
        }

        const auto received = Clock::now();
        last_frame = received;
        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();
        if (!color || !depth) {
          throw std::runtime_error("incomplete RGB-D frameset");
        }

        const double color_timestamp = color.get_timestamp() * 0.001;
        const double depth_timestamp = depth.get_timestamp() * 0.001;
        const int color_domain = color.get_frame_timestamp_domain();
        const int depth_domain = depth.get_frame_timestamp_domain();
        const auto color_number = color.get_frame_number();
        const auto depth_number = depth.get_frame_number();

        if (previous_color && color_number > previous_color + 1) {
          color_gaps += color_number - previous_color - 1;
        }
        if (previous_depth && depth_number > previous_depth + 1) {
          depth_gaps += depth_number - previous_depth - 1;
        }

        std::string reason =
            (previous_color && color_number <= previous_color) ||
                    (previous_depth && depth_number <= previous_depth)
                ? "duplicate frame"
                : timestamps.accept(color_timestamp, depth_timestamp, color_domain,
                                    depth_domain);
        previous_color = color_number;
        previous_depth = depth_number;

        if (!reason.empty()) {
          ++rejected;
          if (reason == "changed_timestamp_domain" ||
              ++consecutive_bad > static_cast<unsigned>(fps * 5)) {
            throw std::runtime_error("camera timestamp failure: " + reason);
          }
          continue;
        }
        consecutive_bad = 0;

        if (warmup) {
          --warmup;
          continue;
        }

        auto aligned =
            align.process(frames).as<rs2::frameset>().get_depth_frame();
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

        auto frame = std::make_shared<orb_live::RGBDFrame>();
        frame->color = cv::Mat(camera.height, camera.width, CV_8UC3,
                               const_cast<void*>(color.get_data()),
                               color.get_stride_in_bytes())
                           .clone();
        frame->depth = cv::Mat(camera.height, camera.width, CV_16UC1,
                               const_cast<void*>(aligned.get_data()),
                               aligned.get_stride_in_bytes())
                           .clone();
        frame->index = captured++;
        frame->color_sec = color_timestamp;
        frame->depth_sec = depth_timestamp;
        frame->color_number = color_number;
        frame->depth_number = depth_number;
        frame->color_domain = color_domain;
        frame->depth_domain = depth_domain;
        frame->received = received;
        frame->enqueued = Clock::now();
        frame->valid_depth =
            static_cast<double>(cv::countNonZero(frame->depth)) / frame->depth.total();

        if (!disk.push(frame)) {
          throw std::runtime_error(
              "recording queue overflow; stopping without evicting recorded frames");
        }
        if (!live.push(std::move(frame))) {
          break;
        }
        if (options.max_frames &&
            captured >= static_cast<std::uint64_t>(options.max_frames)) {
          break;
        }
      }
    } catch (const std::exception& error) {
      capture_error = error.what();
      stopping = true;
    }

    live.close();
    disk.close();
  }

  void RecordLoop() {
    try {
      orb_live::FramePtr frame;
      while (disk.pop_for(frame, std::chrono::milliseconds(100)) || !disk.finished()) {
        if (!frame) {
          continue;
        }
        if (writer.count() % 15 == 0 &&
            std::filesystem::space(options.output).available < 256ULL * 1024 * 1024) {
          throw std::runtime_error("disk free space below 256MiB");
        }
        writer.write(*frame);
        recorded = writer.count();
        frame.reset();
      }
    } catch (const std::exception& error) {
      record_error = error.what();
      stopping = true;
      disk.close();
    }
  }

  void Stop() {
    if (stopped) {
      return;
    }
    stopped = true;
    stopping = true;

    if (capture_thread.joinable()) {
      capture_thread.join();
    }
    live.close();
    disk.close();
    if (record_thread.joinable()) {
      record_thread.join();
    }
    if (streaming) {
      pipeline.stop();
      streaming = false;
    }
  }
};

LiveSource::LiveSource(const Options& options, const CameraModel& camera, int fps)
    : impl_(std::make_unique<Impl>(options, camera, fps)) {}

LiveSource::~LiveSource() { stop(); }

bool LiveSource::next(orb_live::FramePtr& frame, std::chrono::milliseconds timeout) {
  return impl_->live.pop_for(frame, timeout);
}

bool LiveSource::finished() const { return impl_->live.finished(); }

void LiveSource::stop() { impl_->Stop(); }

LiveStats LiveSource::stats() const {
  auto& impl = *impl_;
  return {impl.captured, impl.recorded, impl.rejected, impl.color_gaps, impl.depth_gaps,
          impl.live.dropped()};
}

std::string LiveSource::error() const {
  if (!impl_->stopped) {
    throw std::logic_error("stop live source before reading errors");
  }
  return impl_->capture_error.empty() ? impl_->record_error : impl_->capture_error;
}

}  // namespace ego_occupancy
