#pragma once
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace orb_live {
template<class T> class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity = 2, bool reject_full = false)
        : capacity_(capacity), reject_full_(reject_full) {
        if (!capacity_) throw std::invalid_argument("Queue capacity must be positive");
    }
    bool push(T value) {
        std::optional<T> discarded;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_) return false;
            if (items_.size() == capacity_) {
                if (reject_full_) return false;
                discarded.emplace(std::move(items_.front()));
                items_.pop_front(); ++dropped_;
            }
            items_.push_back(std::move(value));
        }
        // Destroy evicted image storage after releasing the queue lock.
        ready_.notify_one();
        return true;
    }
    bool pop_for(T& value, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait_for(lock, timeout, [&] { return closed_ || !items_.empty(); });
        if (items_.empty()) return false;
        value = std::move(items_.front()); items_.pop_front();
        return true;
    }
    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true; ready_.notify_all();
    }
    std::uint64_t dropped() const {
        std::lock_guard<std::mutex> lock(mutex_); return dropped_;
    }
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_); return items_.size();
    }
    bool finished() const {
        std::lock_guard<std::mutex> lock(mutex_); return closed_ && items_.empty();
    }
private:
    std::size_t capacity_;
    bool reject_full_;
    std::deque<T> items_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    bool closed_ = false;
    std::uint64_t dropped_ = 0;
};

class TimestampGuard {
public:
    explicit TimestampGuard(double max_skew_sec) : max_skew_(max_skew_sec) {
        if (!std::isfinite(max_skew_) || max_skew_ <= 0)
            throw std::invalid_argument("Timestamp tolerance must be finite and positive");
    }
    std::string accept(double color, double depth, int color_domain, int depth_domain) {
        if (!std::isfinite(color) || !std::isfinite(depth)) return "nonfinite_timestamp";
        if (color_domain != depth_domain) return "different_timestamp_domains";
        if (initialized_ && color_domain != domain_) return "changed_timestamp_domain";
        if (initialized_ && (color <= color_ || depth <= depth_)) return "nonincreasing_timestamp";
        if (std::abs(color - depth) > max_skew_) return "pair_skew_exceeded";
        color_ = color; depth_ = depth; domain_ = color_domain; initialized_ = true;
        return {};
    }
private:
    double max_skew_, color_ = 0, depth_ = 0;
    int domain_ = 0;
    bool initialized_ = false;
};
} // namespace orb_live
