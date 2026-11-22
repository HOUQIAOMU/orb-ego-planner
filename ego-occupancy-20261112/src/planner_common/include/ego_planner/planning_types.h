#pragma once
#include <ego_occupancy/map.h>
#include <atomic>
#include <chrono>
#include <stdexcept>

namespace ego_planner {
using Vec3 = Eigen::Vector3d;
using Index = Eigen::Vector3i;
using Clock = std::chrono::steady_clock;
enum class WorkStatus { Ready, Timeout, Cancelled };
struct WorkBudget {
  Clock::time_point deadline = Clock::now() + std::chrono::milliseconds(200);
  const std::atomic<bool>* cancelled = nullptr;  // 调用方保证其生命周期覆盖本轮任务。
  WorkStatus status() const {
    if (cancelled && cancelled->load(std::memory_order_relaxed)) return WorkStatus::Cancelled;
    return Clock::now() >= deadline ? WorkStatus::Timeout : WorkStatus::Ready;
  }
};
struct Roi {
  Index minimum = Index::Zero();
  Index maximum_exclusive = Index::Zero();
};
}  // namespace ego_planner
