#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <utility>

// This side channel changes no ORB public class layout and supports C++11 cores.
namespace ego_pose_bridge {

/// ABI-stable pose record. Field order and types are part of the bridge
/// contract with the patched ORB core; do not reorder or resize.
struct Record {
  bool captured = false;
  bool valid = false;
  int tracking_state = -1;
  std::uint64_t session = 0;
  std::uint64_t map_token = 0;
  std::uint64_t revision = 0;
  std::uint64_t frame_id = 0;
  double timestamp_sec = 0.0;
  std::array<float, 16> T_world_camera{};  // column major, meters
};

using Observer = void (*)(void*, std::uint64_t, const char*);

void Initialize(std::uint64_t session);
void SetObserver(Observer observer, void* context);
void BeginFrame(double timestamp_sec);
void Capture(std::uint64_t frame_id,
             double timestamp_sec,
             const void* map_identity,
             const float* T_world_camera,
             int tracking_state,
             bool pose_set);
Record Last();
bool IsCurrent(const Record& record);
std::uint64_t Revision();

class CorrectionScope {
 public:
  explicit CorrectionScope(const char* reason);
  ~CorrectionScope();

  CorrectionScope(const CorrectionScope&) = delete;
  CorrectionScope& operator=(const CorrectionScope&) = delete;

 private:
  const char* reason_;
};

class CaptureGuard {
 public:
  explicit CaptureGuard(std::function<void()> callback) : callback_(std::move(callback)) {}
  ~CaptureGuard() { callback_(); }

  CaptureGuard(const CaptureGuard&) = delete;
  CaptureGuard& operator=(const CaptureGuard&) = delete;

 private:
  std::function<void()> callback_;
};

}  // namespace ego_pose_bridge
