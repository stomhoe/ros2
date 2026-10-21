#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <thread>
#include <time.h>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "block_plugin_engine/execution_config.hpp"
#include "block_plugin_engine/iblock_scheduler.hpp"
#include "block_plugin_engine/itimer_block.hpp"

namespace bpe::test_support {

using namespace std::chrono_literals;

constexpr std::size_t MAX_SAMPLES = 20'000;
constexpr std::size_t MAX_EVENTS = 40'000;

constexpr double INITIAL_A = 1.0;
constexpr double INITIAL_B = 0.25;

inline volatile double work_result = 0.0;

inline std::int64_t to_nanoseconds(const timespec& time) {
  return static_cast<std::int64_t>(time.tv_sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.tv_nsec);
}

inline std::int64_t now_nanoseconds() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return to_nanoseconds(now);
}

inline void floating_point_work(std::size_t iterations, double& a, double& b) {
  for (std::size_t i = 0; i < iterations; ++i) {
    a = a + b;
    b = b * 2.0;
    a = a - b;
    b = b * 0.5;
    a = a + b;
  }
}

inline std::int64_t measure_work_time(std::size_t iterations) {
  double a = INITIAL_A;
  double b = INITIAL_B;

  const std::int64_t start = now_nanoseconds();
  floating_point_work(iterations, a, b);
  const std::int64_t end = now_nanoseconds();

  work_result = a + b;

  return end - start;
}

inline std::size_t calibrate_iterations(std::chrono::nanoseconds target) {
  std::size_t iterations = 10'000;

  measure_work_time(iterations);

  for (std::size_t attempt = 0; attempt < 8; ++attempt) {
    const std::int64_t elapsed = measure_work_time(iterations);

    if (elapsed > 0) {
      double ratio =
        static_cast<double>(target.count()) / static_cast<double>(elapsed);

      if (ratio > 0.85 && ratio < 1.15) {
        break;
      }

      if (ratio < 0.25) {
        ratio = 0.25;
      } else if (ratio > 4.0) {
        ratio = 4.0;
      }

      iterations =
        static_cast<std::size_t>(static_cast<double>(iterations) * ratio);

      if (iterations == 0) {
        iterations = 1;
      }
    }
  }

  return iterations;
}

class ExecutionLog {
public:
  void record(int block_id) {
    const std::size_t index =
      count_.fetch_add(1, std::memory_order_relaxed);

    if (index < events_.size()) {
      events_[index] = block_id;
    }
  }

  std::size_t count() const {
    return count_.load(std::memory_order_acquire);
  }

  int event(std::size_t index) const {
    return events_[index];
  }

private:
  std::array<int, MAX_EVENTS> events_{};
  std::atomic<std::size_t> count_{0};
};

class RecordingTimerBlock : public ITimerBlock {
public:
  RecordingTimerBlock(std::size_t min_iterations,
                      std::size_t max_iterations,
                      std::size_t first_iterations = 0,
                      std::uint32_t seed = 12345,
                      ExecutionLog* log = nullptr,
                      int block_id = 0)
    : min_iterations_(min_iterations),
      max_iterations_(max_iterations),
      first_iterations_(first_iterations),
      generator_(seed),
      distribution_(min_iterations, max_iterations),
      log_(log),
      block_id_(block_id) {}

  ExecutionResult execute() override {
    const std::size_t index =
      count_.fetch_add(1, std::memory_order_relaxed);

    const std::int64_t start = now_nanoseconds();

    if (index < timestamps_.size()) {
      timestamps_[index] = start;
    }

    if (log_ != nullptr) {
      log_->record(block_id_);
    }

    std::size_t iterations = distribution_(generator_);

    if (index == 0 && first_iterations_ > 0) {
      iterations = first_iterations_;
    }

    double a = value_a_;
    double b = value_b_;

    floating_point_work(iterations, a, b);

    value_a_ = a;
    value_b_ = b;

    const std::int64_t end = now_nanoseconds();

    if (index < execution_times_.size()) {
      execution_times_[index] = end - start;
      iterations_[index] = iterations;
    }

    return ExecutionResult::SUCCESS;
  }

  std::size_t count() const {
    return count_.load(std::memory_order_acquire);
  }

  std::int64_t timestamp_ns(std::size_t index) const {
    return timestamps_[index];
  }

  std::chrono::nanoseconds execution_time(std::size_t index) const {
    return std::chrono::nanoseconds{execution_times_[index]};
  }

  std::size_t iterations(std::size_t index) const {
    return iterations_[index];
  }

  double value_a() const {
    return value_a_;
  }

  double value_b() const {
    return value_b_;
  }

private:
  std::array<std::int64_t, MAX_SAMPLES> timestamps_{};
  std::array<std::int64_t, MAX_SAMPLES> execution_times_{};
  std::array<std::size_t, MAX_SAMPLES> iterations_{};

  std::atomic<std::size_t> count_{0};

  std::size_t min_iterations_;
  std::size_t max_iterations_;
  std::size_t first_iterations_;

  std::mt19937 generator_;
  std::uniform_int_distribution<std::size_t> distribution_;

  ExecutionLog* log_;
  int block_id_;

  double value_a_{INITIAL_A};
  double value_b_{INITIAL_B};
};

inline bool wait_for_samples(const RecordingTimerBlock& block,
                             std::size_t expected,
                             std::chrono::milliseconds timeout) {
  const auto limit = std::chrono::steady_clock::now() + timeout;

  while (block.count() < expected &&
         std::chrono::steady_clock::now() < limit) {
    std::this_thread::sleep_for(1ms);
  }

  return block.count() >= expected;
}

inline bool wait_for_events(const ExecutionLog& log,
                            std::size_t expected,
                            std::chrono::milliseconds timeout) {
  const auto limit = std::chrono::steady_clock::now() + timeout;

  while (log.count() < expected &&
         std::chrono::steady_clock::now() < limit) {
    std::this_thread::sleep_for(1ms);
  }

  return log.count() >= expected;
}

inline SchedulerConfig scheduler_config(
  std::chrono::nanoseconds spin_before_deadline) {
  YAML::Node config;
  config["realtime"]["enabled"] = false;
  config["timing"]["spin_before_deadline"] =
    std::to_string(spin_before_deadline.count()) + "ns";
  return config;
}

inline ExecutionConfig timer_config(std::chrono::nanoseconds period) {
  ExecutionConfig config;
  config.policy = ExecutionPolicy::TIMER;
  config.period = period;
  return config;
}

} // namespace bpe::test_support
