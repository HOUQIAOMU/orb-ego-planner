#include "capture_backend.h"
#include "test_support.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <future>
#include <sys/stat.h>
#include <thread>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace ego_occupancy;

namespace {

std::atomic<unsigned> available{0};
std::atomic<bool> camera_stopped{false};
bool provoke_timeout = false;

class FakeCamera final : public CaptureBackend {
 public:
  void writeDeviceMetadata(cv::FileStorage& metadata) const override {
    metadata << "serial" << "fake";
  }

  std::optional<CaptureMetadata> read(int) override {
    if (index_ == available) {
      std::this_thread::sleep_for(1ms);
      return std::nullopt;
    }
    ++index_;
    const double timestamp = index_ / 20.0;
    const auto received = Clock::now() - (provoke_timeout ? 6s : 0s);
    return CaptureMetadata{timestamp, timestamp, index_, index_, 1, 1, received};
  }

  void copyImages(orb_live::RGBDFrame& output) override {
    output.color = cv::Mat(8, 8, CV_8UC3, cv::Scalar(3, 2, 1));
    output.depth = cv::Mat(8, 8, CV_16UC1, cv::Scalar(1000));
  }

  void stop() override { camera_stopped = true; }

  std::string frameTimeoutDetails() const override { return "test backend produced no frames"; }

 private:
  unsigned index_ = 0;
};

template<class Predicate>
void WaitFor(Predicate done) {
  const auto deadline = Clock::now() + 2s;
  while (!done() && Clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  CHECK(done());
}

struct Scratch {
  Scratch() {
    std::string pattern = (fs::temp_directory_path() / "ego-live-source-XXXXXX").string();
    CHECK(mkdtemp(pattern.data()));
    path = pattern;
  }
  ~Scratch() {
    std::error_code error;
    fs::remove_all(path, error);
  }
  fs::path path;
};

Options Settings(const fs::path& root) {
  available = 0;
  camera_stopped = false;
  provoke_timeout = false;
  Options opts;
  opts.camera_backend = CameraBackend::Orbbec;
  opts.warmup = 0;
  opts.record_hz = 60;
  opts.output = root;
  opts.camera = root / "camera.yaml";
  fs::create_directories(root);
  std::ofstream(opts.camera) << "test camera metadata";
  return opts;
}

void RecordingFailureKeepsCameraLive(const fs::path& root) {
  LiveSource source(Settings(root), CameraModel{}, 60);
  available = 1;
  orb_live::FramePtr frame;
  CHECK(source.next(frame, 1s) && frame->index == 0);
  WaitFor([&] { return source.stats().recorded == 1; });

  // A directory at the second depth filename forces a pair commit failure.
  fs::create_directory(root / "rgbd/depth/000001.png");
  available = 2;
  CHECK(source.next(frame, 1s) && frame->index == 1);
  WaitFor([&] { return source.stats().recording_failed; });
  available = 3;
  CHECK(source.next(frame, 1s) && frame->index == 2);
  CHECK(!camera_stopped);
  source.stop();

  CHECK(camera_stopped && source.error().empty());
  CHECK(!source.recordingError().empty());
  CHECK(source.stats().recorded == 1);
  CHECK(fs::is_regular_file(root / "rgbd/rgb/000000.png"));
  CHECK(fs::is_regular_file(root / "rgbd/depth/000000.png"));
  CHECK(!fs::exists(root / "rgbd/rgb/000001.png"));
  std::ifstream manifest(root / "rgbd/associations.txt");
  std::string row;
  CHECK(std::getline(manifest, row) && row.find("000000.png") != std::string::npos);
  CHECK(!std::getline(manifest, row));
}

void DisabledRecordingKeepsMetadata(const fs::path& root) {
  Options opts = Settings(root);
  opts.record_hz = 0;
  LiveSource source(opts, CameraModel{}, 60);
  available = 1;
  orb_live::FramePtr frame;
  CHECK(source.next(frame, 1s) && frame->index == 0);
  source.stop();
  CHECK(camera_stopped && source.error().empty() && source.recordingError().empty());
  CHECK(source.stats().recorded == 0);
  CHECK(fs::is_regular_file(root / "rgbd/device.yml"));
  CHECK(fs::is_regular_file(root / "rgbd/camera.yaml"));
  CHECK(!fs::exists(root / "rgbd/rgb") && !fs::exists(root / "rgbd/associations.txt"));
}


void InitializationFailureKeepsCameraLive(const fs::path& root) {
  Options opts = Settings(root);
  fs::create_directories(root / "rgbd");
  std::ofstream(root / "rgbd/frames.csv") << "existing capture";
  LiveSource source(opts, CameraModel{}, 60);
  CHECK(source.stats().recording_failed);
  available = 1;
  orb_live::FramePtr frame;
  CHECK(source.next(frame, 1s) && frame->index == 0);
  source.stop();
  CHECK(source.error().empty() && !source.recordingError().empty());
  std::ifstream manifest(root / "rgbd/frames.csv");
  std::string line;
  CHECK(std::getline(manifest, line) && line == "existing capture");
}

void TimeoutPreservesBackendDiagnostics(const fs::path& root) {
  Options opts = Settings(root);
  opts.record_hz = 0;
  provoke_timeout = true;
  LiveSource source(opts, CameraModel{}, 60);
  available = 1;
  orb_live::FramePtr frame;
  CHECK(source.next(frame, 1s));
  WaitFor([&] { return source.finished(); });
  source.stop();
  CHECK(source.error().find("test backend produced no frames") != std::string::npos);
  CHECK(source.recordingError().empty());
}

void CameraStopsWhileRecorderIsBlocked(const fs::path& root) {
  LiveSource source(Settings(root), CameraModel{}, 60);
  const auto pending_png = root / "rgbd/rgb/000000.partial.png";
  // PNG's file open blocks on the FIFO until this test supplies a reader.
  CHECK(mkfifo(pending_png.c_str(), 0600) == 0);
  available = 1;
  orb_live::FramePtr frame;
  CHECK(source.next(frame, 1s));
  WaitFor([&] { return source.stats().record_eligible == 1; });
  auto stop = std::async(std::launch::async, [&] { source.stop(); });

  const auto deadline = Clock::now() + 2s;
  while (!camera_stopped && Clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  const bool stopped_before_writer = camera_stopped;
  const int reader = open(pending_png.c_str(), O_RDONLY | O_NONBLOCK);
  CHECK(reader >= 0);
  stop.get();
  close(reader);
  CHECK(stopped_before_writer && source.error().empty());
}

}  // namespace

namespace ego_occupancy {
std::unique_ptr<CaptureBackend> openOrbbec(const Options&, const CameraModel&, int) {
  return std::make_unique<FakeCamera>();
}
}  // namespace ego_occupancy

int main() {
  Scratch scratch;
  RecordingFailureKeepsCameraLive(scratch.path / "writer-fault");
  DisabledRecordingKeepsMetadata(scratch.path / "disabled");
  InitializationFailureKeepsCameraLive(scratch.path / "init-fault");
  CameraStopsWhileRecorderIsBlocked(scratch.path / "blocked-recorder");
  TimeoutPreservesBackendDiagnostics(scratch.path / "timeout");
}
