#include "live_source.h"

#include "capture_backend.h"

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
/*
它实际上把两类工作放在了同一个类里：用 RealSense SDK 把相机数据变成一对可用的 RGB-D 图像，
再用项目自己的线程、队列和录制器分发这对图像。 迁移到 Orbbec 时，最合适的切分点是 RGBDFrame。
它已经装的是自有内存的 OpenCV 图像和时间戳，下游可以继续用同一个接口。
*/

namespace ego_occupancy {

bool liveBackendAvailable(CameraBackend backend) {
#ifdef EGO_CAMERA_REALSENSE
  if (backend == CameraBackend::RealSense) return true;
#endif
#ifdef EGO_CAMERA_ORBBEC
  if (backend == CameraBackend::Orbbec) return true;
#endif
  (void)backend;
  return false;
}

namespace {

std::unique_ptr<CaptureBackend> OpenBackend([[maybe_unused]] const Options& options,
                                            [[maybe_unused]] const CameraModel& camera,
                                            [[maybe_unused]] int fps) {
#ifdef EGO_CAMERA_REALSENSE
  if (options.camera_backend == CameraBackend::RealSense) return openRealSense(options, camera, fps);
#endif
#ifdef EGO_CAMERA_ORBBEC
  if (options.camera_backend == CameraBackend::Orbbec) return openOrbbec(options, camera, fps);
#endif
  throw std::invalid_argument("selected live camera backend was not built");
}

}  // namespace

struct LiveSource::Impl {
  Options options;
  CameraModel camera;
  int fps = 0;
  std::unique_ptr<CaptureBackend> backend;
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
  bool stopped = false;

  Impl(const Options& source_options, const CameraModel& camera_model, int frame_rate)
      : options(source_options),
        camera(camera_model),
        fps(frame_rate),
        writer(options.output / "rgbd") {
    try {
      backend = OpenBackend(options, camera, fps);
      const bool realsense = options.camera_backend == CameraBackend::RealSense;
      std::filesystem::copy_file(options.camera,
                                 options.output / "rgbd" / (realsense ? "D435i.yaml" : "Gemini.yaml"));
      cv::FileStorage metadata((options.output / "rgbd/device.yml").string(),
                               cv::FileStorage::WRITE);
      if (!metadata.isOpened()) throw std::runtime_error("cannot write camera metadata");
      metadata << "backend" << (realsense ? "realsense" : "orbbec");
      backend->writeDeviceMetadata(metadata);
      metadata.release();

      record_thread = std::thread([this] { RecordLoop(); });
      capture_thread = std::thread([this] { CaptureLoop(); });
    } catch (...) {
      Stop();
      throw;
    }
  }

  void CaptureLoop() {
    try {
      // 主逻辑
      orb_live::TimestampGuard timestamps(0.010);
      int warmup = options.warmup;
      std::uint64_t previous_color = 0;
      std::uint64_t previous_depth = 0;
      auto last_frame = Clock::now();
      unsigned consecutive_bad = 0;

      while (!stopping) {
        const auto sample = backend->read(200);
        if (!sample) {
          if (std::chrono::duration<double>(Clock::now() - last_frame).count() > 5)
            throw std::runtime_error("no RGB-D frames for 5 seconds");
          continue;
        }

        const auto [color_timestamp, depth_timestamp, color_number, depth_number,
                    color_domain, depth_domain, received] = *sample;
        last_frame = received;

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

        auto frame = std::make_shared<orb_live::RGBDFrame>();
        backend->copyImages(*frame);
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
    if (backend) {
      try {
        backend->stop();
      } catch (const std::exception& error) {
        if (capture_error.empty()) capture_error = error.what();
      }
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
