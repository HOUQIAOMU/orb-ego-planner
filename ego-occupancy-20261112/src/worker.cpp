#include <ego_occupancy/worker.h>

#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace ego_occupancy {

struct MappingWorker::Impl {
  struct Job {
    cv::Mat depth;
    ego_pose_bridge::Record record;
    std::uint64_t index = 0;
    std::uint64_t generation = 0;
    TimePoint received;
  };

  OccupancyMapper mapper;
  DepthProjector projector;
  mutable std::mutex gate;
  mutable std::mutex queue_mutex;
  std::condition_variable ready;
  std::condition_variable completed;
  std::deque<Job> queue;
  std::thread thread;
  WorkerStats stats;
  std::uint64_t generation = 0;
  std::uint64_t last_completed = 0;
  std::uint64_t last_submitted = 0;
  bool closing = false;
  bool closed = false;
  bool need_reset = true;
  bool have_completed = false;
  bool have_submitted = false;
  SourceStamp active;
  std::ofstream log;

  Impl(const Config& config,
       const CameraModel& camera,
       const std::filesystem::path& diagnostics)
      : mapper(config), projector(camera) {
    if (!diagnostics.empty()) {
      if (std::filesystem::exists(diagnostics)) {
        throw std::runtime_error("diagnostics already exist");
      }
      log.open(diagnostics);
      log.exceptions(std::ios::failbit | std::ios::badbit);
      log << "capture_index,orb_frame,source_map,source_revision,accepted,published,"
             "valid_rays,updated_voxels,fuse_ms,inflate_ms,publish_ms,age_ms,reason\n"
          << std::setprecision(9);
    }

    ego_pose_bridge::SetObserver(
        [](void* context, std::uint64_t, const char* reason) {
          static_cast<Impl*>(context)->Invalidate(reason);
        },
        this);

    try {
      thread = std::thread([this] { Run(); });
    } catch (...) {
      ego_pose_bridge::SetObserver(nullptr, nullptr);
      throw;
    }
  }

  void Invalidate(const std::string& reason) {
    std::lock_guard<std::mutex> lock(gate);
    ++generation;
    need_reset = true;
    ++stats.source_events;
    mapper.invalidate(reason);
  }

  void Run() {
    while (true) {
      Job job;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        ready.wait(lock, [this] { return closed || !queue.empty(); });
        if (closed) {
          break;
        }
        job = std::move(queue.front());
        queue.pop_front();
      }

      UpdateResult result;
      try {
        bool permitted = false;
        {
          std::lock_guard<std::mutex> lock(gate);
          permitted = !closing && job.generation == generation &&
                      ego_pose_bridge::IsCurrent(job.record);
          if (permitted) {
            const SourceStamp source{
                job.record.session, job.record.map_token, job.record.revision};
            if (need_reset || source != active) {
              const Eigen::Map<const Eigen::Matrix4f> transform(
                  job.record.T_world_camera.data());
              mapper.reset(source, transform.block<3, 1>(0, 3).cast<double>());
              active = source;
              need_reset = false;
            }
          }
        }

        if (permitted) {
          Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
          pose.matrix() =
              Eigen::Map<const Eigen::Matrix4f>(job.record.T_world_camera.data())
                  .cast<double>();
          const auto observation = projector.project(
              job.depth, pose,
              {job.record.session, job.record.map_token, job.record.revision},
              job.index, job.record.timestamp_sec, job.received);
          result = mapper.integrate(observation);
        } else {
          result.reason = "stale source generation";
        }
      } catch (const std::exception& error) {
        result.reason = "mapping exception";
        std::lock_guard<std::mutex> lock(gate);
        stats.error = error.what();
        ++generation;
        need_reset = true;
        mapper.invalidate(stats.error);
      }

      const double age =
          std::chrono::duration<double, std::milli>(Clock::now() - job.received).count();
      try {
        if (log) {
          log << job.index << ',' << job.record.frame_id << ',' << job.record.map_token << ','
              << job.record.revision << ',' << result.accepted << ',' << result.published
              << ',' << result.valid_rays << ',' << result.updated_voxels << ','
              << result.fuse_ms << ',' << result.inflate_ms << ',' << result.publish_ms
              << ',' << age << ',' << result.reason << '\n';
          log.flush();
        }
      } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(gate);
        stats.error = error.what();
        ++generation;
        need_reset = true;
        mapper.invalidate("mapping diagnostics write failure");
      }

      {
        std::lock_guard<std::mutex> lock(gate);
        if (result.accepted) {
          ++stats.fused;
        } else {
          ++stats.rejected;
        }
        if (result.accepted && !result.published) {
          ++stats.publish_skipped;
        }
        stats.last_fuse_ms = result.fuse_ms;
        stats.last_inflate_ms = result.inflate_ms;
        stats.last_publish_ms = result.publish_ms;
        stats.last_age_ms = age;
        last_completed = job.index;
        have_completed = true;
      }
      completed.notify_all();
    }
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> lock(gate);
      if (closing) {
        return;
      }
      closing = true;
      ++generation;
      mapper.invalidate("input stopped");
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      closed = true;
      queue.clear();
    }
    ready.notify_all();

    if (thread.joinable()) {
      thread.join();
    }
    ego_pose_bridge::SetObserver(nullptr, nullptr);
    completed.notify_all();
  }
};

MappingWorker::MappingWorker(const Config& config,
                             const CameraModel& camera,
                             const std::filesystem::path& diagnostics)
    : impl_(std::make_unique<Impl>(config, camera, diagnostics)) {}

MappingWorker::~MappingWorker() { stop(); }

bool MappingWorker::submit(const cv::Mat& depth,
                           const ego_pose_bridge::Record& record,
                           std::uint64_t capture_index,
                           TimePoint received) {
  auto& impl = *impl_;
  std::lock_guard<std::mutex> lock(impl.gate);
  if (impl.closing || !ego_pose_bridge::IsCurrent(record) ||
      (impl.have_submitted && capture_index <= impl.last_submitted)) {
    ++impl.stats.rejected;
    return false;
  }

  Impl::Job job{depth.clone(), record, capture_index, impl.generation, received};
  {
    std::lock_guard<std::mutex> queue_lock(impl.queue_mutex);
    if (impl.queue.size() == 2) {
      impl.queue.pop_front();
      ++impl.stats.dropped;
    }
    impl.queue.push_back(std::move(job));
  }

  impl.have_submitted = true;
  impl.last_submitted = capture_index;
  ++impl.stats.submitted;
  impl.ready.notify_one();
  return true;
}

bool MappingWorker::waitFor(std::uint64_t capture_index,
                            std::chrono::milliseconds timeout) {
  auto& impl = *impl_;
  std::unique_lock<std::mutex> lock(impl.gate);
  return impl.completed.wait_for(lock, timeout, [&impl, capture_index] {
           return (impl.have_completed && impl.last_completed >= capture_index) ||
                  impl.closing;
         }) &&
         !impl.closing;
}

void MappingWorker::trackingLost(const std::string& reason) { impl_->Invalidate(reason); }

void MappingWorker::stop() { impl_->Stop(); }

MapReader MappingWorker::reader() const { return impl_->mapper.reader(); }

WorkerStats MappingWorker::stats() const {
  std::lock_guard<std::mutex> lock(impl_->gate);
  return impl_->stats;
}

}  // namespace ego_occupancy
