#include "pose_bridge.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <stdexcept>

namespace ego_pose_bridge {
namespace {

struct State {
  std::mutex mutex;
  std::mutex observer_mutex;
  std::uint64_t session = 0;
  std::uint64_t revision = 0;
  std::uint64_t next_map = 0;
  unsigned corrections = 0;
  std::map<const void*, std::uint64_t> maps;
  Observer observer = nullptr;
  void* context = nullptr;
};

// Retained until process exit: upstream ORB may still own backend workers at exit.
State& StateInstance() {
  static auto* state = new State;
  return *state;
}

thread_local Record record;

void Notify(std::uint64_t revision, const char* reason) {
  auto& state = StateInstance();
  std::lock_guard<std::mutex> lock(state.observer_mutex);
  if (state.observer) {
    state.observer(state.context, revision, reason);
  }
}

}  // namespace

void Initialize(std::uint64_t session) {
  if (!session) {
    throw std::invalid_argument("bridge session must be nonzero");
  }

  auto& state = StateInstance();
  std::lock_guard<std::mutex> lock(state.mutex);
  if (state.corrections) {
    throw std::logic_error("bridge initialization during correction");
  }

  state.session = session;
  state.revision = 1;
  state.next_map = 0;
  state.maps.clear();
  record = Record{};
}

void SetObserver(Observer observer, void* context) {
  auto& state = StateInstance();
  std::lock_guard<std::mutex> lock(state.observer_mutex);
  state.observer = observer;
  state.context = context;
}

void BeginFrame(double timestamp) {
  record = Record{};
  record.timestamp_sec = timestamp;
}

void Capture(std::uint64_t frame,
             double timestamp,
             const void* map,
             const float* T_world_camera,
             int tracking,
             bool pose_set) {
  const double expected_timestamp = record.timestamp_sec;
  record = Record{};
  record.captured = true;
  record.frame_id = frame;
  record.timestamp_sec = timestamp;
  record.tracking_state = tracking;

  auto& state = StateInstance();
  std::lock_guard<std::mutex> lock(state.mutex);
  record.session = state.session;
  record.revision = state.revision;

  if (map) {
    auto it = state.maps.find(map);
    if (it == state.maps.end()) {
      it = state.maps.emplace(map, ++state.next_map).first;
    }
    record.map_token = it->second;
  }

  if (T_world_camera) {
    std::copy(T_world_camera, T_world_camera + 16, record.T_world_camera.begin());
  }

  bool finite = T_world_camera;
  for (const float value : record.T_world_camera) {
    finite = finite && std::isfinite(value);
  }

  record.valid = state.session && state.corrections == 0 && map && pose_set &&
                 tracking == 2 && T_world_camera && finite && std::isfinite(timestamp) &&
                 timestamp == expected_timestamp;
}

Record Last() { return record; }

bool IsCurrent(const Record& record) {
  auto& state = StateInstance();
  std::lock_guard<std::mutex> lock(state.mutex);
  return record.valid && state.corrections == 0 && record.session == state.session &&
         record.revision == state.revision;
}

std::uint64_t Revision() {
  auto& state = StateInstance();
  std::lock_guard<std::mutex> lock(state.mutex);
  return state.revision;
}

CorrectionScope::CorrectionScope(const char* reason) : reason_(reason) {
  auto& state = StateInstance();
  std::uint64_t revision = 0;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    ++state.corrections;
    revision = ++state.revision;
    state.maps.clear();
  }
  Notify(revision, reason_);
}

CorrectionScope::~CorrectionScope() {
  auto& state = StateInstance();
  std::uint64_t revision = 0;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    --state.corrections;
    revision = ++state.revision;
  }
  Notify(revision, reason_);
}

}  // namespace ego_pose_bridge
