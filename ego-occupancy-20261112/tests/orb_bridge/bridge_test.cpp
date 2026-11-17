#include "pose_bridge.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace ego_pose_bridge;

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      throw std::runtime_error(#condition);                                    \
    }                                                                          \
  } while (false)

int main() {
  try {
    Initialize(42);
    std::atomic<int> notices{0};
    SetObserver(
        [](void* context, std::uint64_t, const char*) {
          ++*static_cast<std::atomic<int>*>(context);
        },
        &notices);

    std::array<float, 16> transform{};
    transform[0] = transform[5] = transform[10] = transform[15] = 1;
    transform[12] = 2;
    int map_identity = 0;

    BeginFrame(1.0);
    CHECK(!Last().valid);
    Capture(10, 1.0, &map_identity, transform.data(), 2, true);
    auto first = Last();
    CHECK(first.valid && first.frame_id == 10 && first.session == 42 &&
          first.T_world_camera[12] == 2);
    CHECK(IsCurrent(first));

    BeginFrame(2.0);
    CHECK(!Last().valid);
    // An early return cannot reuse frame 10.
    Capture(11, 2.0, &map_identity, transform.data(), 3, true);
    CHECK(!Last().valid);

    {
      CorrectionScope correction("loop");
      CHECK(!IsCurrent(first));
      BeginFrame(3.0);
      Capture(12, 3.0, &map_identity, transform.data(), 2, true);
      CHECK(!Last().valid);
    }

    BeginFrame(4.0);
    Capture(13, 4.0, &map_identity, transform.data(), 2, true);
    auto second = Last();
    CHECK(IsCurrent(second) && second.revision != first.revision);
    CHECK(notices.load() == 2);

    std::thread reset_thread([&] { CorrectionScope correction("reset"); });
    reset_thread.join();
    CHECK(!IsCurrent(second));
    CHECK(notices.load() == 4);

    BeginFrame(5.0);
    Capture(14, 4.0, &map_identity, transform.data(), 2, true);
    CHECK(!Last().valid);

    SetObserver(nullptr, nullptr);
    std::cout << "frame binding, early return, correction, reset and observer passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
