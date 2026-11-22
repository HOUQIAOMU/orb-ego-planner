#include <ego_occupancy/display.h>

#include <dlfcn.h>
#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

static std::atomic<bool> schedule_display{false};

// Exercise a legal schedule deterministically: the first spawned thread finishes
// before pthread_create returns. This exposes construction-time accesses to
// not-yet-initialized members without mocking the display implementation.
extern "C" int pthread_create(pthread_t* thread,
                              const pthread_attr_t* attributes,
                              void* (*function)(void*),
                              void* argument) {
  using Create = int (*)(pthread_t*, const pthread_attr_t*, void* (*)(void*), void*);
  static auto real_create =
      reinterpret_cast<Create>(dlsym(RTLD_NEXT, "pthread_create"));

  // ARM OpenBLAS starts background workers before main; do not intercept those.
  if (!schedule_display.exchange(false)) {
    return real_create(thread, attributes, function, argument);
  }

  struct Start {
    void* (*function)(void*);
    void* argument;
    std::atomic<bool> done{false};
  } start{function, argument};

  const int result = real_create(
      thread, attributes,
      [](void* context) -> void* {
        auto* start = static_cast<Start*>(context);
        void* result = start->function(start->argument);
        start->done.store(true, std::memory_order_release);
        return result;
      },
      &start);
  if (!result) {
    while (!start.done.load(std::memory_order_acquire)) {
      usleep(100);
    }
  }
  return result;
}

int main() {
  try {
    setenv("DISPLAY", ":9876", 1);
    setenv("PANGOLIN_WINDOW_URI", "ego_test_invalid://", 1);

    ego_occupancy::Config config;
    config.size = ego_occupancy::Vec3(1, 1, 1);
    config.resolution = 0.1;
    ego_occupancy::CameraModel camera;
    camera.width = 640;
    camera.height = 480;
    camera.fx = camera.fy = 500;

    ego_occupancy::OccupancyMapper mapper(config);
    schedule_display = true;
    ego_occupancy::OccupancyDisplay display(mapper.reader(), camera, {});
    if (!display.closed() || display.error().empty()) {
      throw std::runtime_error("display startup failure was lost during construction");
    }
    display.stop();
    std::cout << "early display thread failure safely retained\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
