#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <thread>
#include <time.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "block_plugin_engine/execution_config.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "detail/bpe_simple_scheduler.hpp"

namespace bpe::test {
namespace {

using namespace std::chrono_literals;

constexpr std::size_t SAMPLE_COUNT = 128;
constexpr std::size_t MAX_SAMPLES = 256;
constexpr std::size_t MAX_EVENTS = 512;

constexpr auto PERIOD = 20ms;

constexpr std::int64_t PERIOD_NS =
    std::chrono::duration_cast<std::chrono::nanoseconds>(PERIOD).count();

constexpr auto NORMAL_WORK_TIME = 2ms;
constexpr auto VARIABLE_WORK_MAX_TIME = 8ms;
constexpr auto OVERRUN_WORK_TIME = 35ms;
constexpr auto OVERLOAD_WORK_TIME = 15ms;

constexpr auto INTERVAL_TOLERANCE = 3ms;
constexpr auto PHASE_TOLERANCE = 3ms;

constexpr auto TEST_TIMEOUT = 10s;

constexpr std::uint32_t RANDOM_SEED = 12345;

constexpr double INITIAL_A = 1.0;
constexpr double INITIAL_B = 0.25;

volatile double work_result = 0.0;

std::int64_t to_nanoseconds(const timespec& time) {
  return static_cast<std::int64_t>(time.tv_sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.tv_nsec);
}

std::int64_t now_nanoseconds() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);

  return to_nanoseconds(now);
}

void floating_point_work(std::size_t iterations, double& a, double& b) {
  for (std::size_t i = 0; i < iterations; ++i) {
    a = a + b;
    b = b * 2.0;
    a = a - b;
    b = b * 0.5;
    a = a + b;
  }
}

std::int64_t measure_work_time(std::size_t iterations) {
  double a = INITIAL_A;
  double b = INITIAL_B;

  const std::int64_t start = now_nanoseconds();

  floating_point_work(iterations, a, b);

  const std::int64_t end = now_nanoseconds();

  //
  // The values are externally observable so the compiler cannot simply
  // discard the calculation.
  //
  work_result = a + b;

  return end - start;
}

std::size_t calibrate_iterations(std::chrono::nanoseconds target) {
  std::size_t iterations = 10'000;

  //
  // Warm up the code path before taking calibration measurements.
  //
  measure_work_time(iterations);

  for (std::size_t attempt = 0; attempt < 8; ++attempt) {
    const std::int64_t elapsed = measure_work_time(iterations);

    if (elapsed > 0) {
      const double ratio = static_cast<double>(target.count()) / static_cast<double>(elapsed);

      if (ratio > 0.85 && ratio < 1.15) {
        break;
      }

      double bounded_ratio = ratio;

      if (bounded_ratio < 0.25) {
        bounded_ratio = 0.25;
      } else if (bounded_ratio > 4.0) {
        bounded_ratio = 4.0;
      }

      iterations = static_cast<std::size_t>(static_cast<double>(iterations) * bounded_ratio);

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
    const std::size_t index = count_.fetch_add(1, std::memory_order_relaxed);

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
                      std::uint32_t seed = RANDOM_SEED,
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
    const std::size_t index = count_.fetch_add(1, std::memory_order_relaxed);

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

bool wait_for_samples(const RecordingTimerBlock& block,
                      std::size_t expected,
                      std::chrono::milliseconds timeout) {
  const auto limit = std::chrono::steady_clock::now() + timeout;

  while (block.count() < expected && std::chrono::steady_clock::now() < limit) {
    std::this_thread::sleep_for(1ms);
  }

  return block.count() >= expected;
}

bool wait_for_events(const ExecutionLog& log,
                     std::size_t expected,
                     std::chrono::milliseconds timeout) {
  const auto limit = std::chrono::steady_clock::now() + timeout;

  while (log.count() < expected && std::chrono::steady_clock::now() < limit) {
    std::this_thread::sleep_for(1ms);
  }

  return log.count() >= expected;
}

SchedulerConfig normal_test_config() {
  return YAML::Load(R"(
realtime:
  enabled: false

timing:
  spin_before_deadline: 500us
)");
}

ExecutionConfig timer_config(std::chrono::nanoseconds period) {
  ExecutionConfig config;
  config.policy = ExecutionPolicy::TIMER;
  config.period = period;

  return config;
}

void check_block_result(const RecordingTimerBlock& block) {
  EXPECT_DOUBLE_EQ(block.value_a(), INITIAL_A);
  EXPECT_DOUBLE_EQ(block.value_b(), INITIAL_B);
}

void check_periodic_timing(const RecordingTimerBlock& block) {
  const std::int64_t reference = block.timestamp_ns(0);

  for (std::size_t i = 1; i < SAMPLE_COUNT; ++i) {
    const auto interval =
        std::chrono::nanoseconds{block.timestamp_ns(i) - block.timestamp_ns(i - 1)};

    EXPECT_GE(interval, PERIOD - INTERVAL_TOLERANCE) << "Sample " << i << " interval too short";

    EXPECT_LE(interval, PERIOD + INTERVAL_TOLERANCE) << "Sample " << i << " interval too long";

    const std::int64_t expected = reference + static_cast<std::int64_t>(i) * PERIOD_NS;

    const auto phase_error = std::chrono::nanoseconds{block.timestamp_ns(i) - expected};

    EXPECT_GE(phase_error, -PHASE_TOLERANCE) << "Sample " << i << " is too early";

    EXPECT_LE(phase_error, PHASE_TOLERANCE) << "Sample " << i << " is too late";
  }
}

TEST(BPESimpleSchedulerTest, PeriodicExecutionMaintainsPhase) {
  const std::size_t work_iterations = calibrate_iterations(NORMAL_WORK_TIME);

  EXPECT_DOUBLE_EQ(work_result, INITIAL_A + INITIAL_B);

  BPE_SimpleScheduler scheduler;

  RecordingTimerBlock block{work_iterations, work_iterations};

  ASSERT_EQ(scheduler.configure(normal_test_config()), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.add("timer", block, timer_config(PERIOD)), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.start(), SchedulerResult::SUCCESS);

  ASSERT_TRUE(wait_for_samples(
      block, SAMPLE_COUNT, std::chrono::duration_cast<std::chrono::milliseconds>(TEST_TIMEOUT)));

  ASSERT_EQ(scheduler.stop(), SchedulerResult::SUCCESS);

  check_periodic_timing(block);
  check_block_result(block);
}

TEST(BPESimpleSchedulerTest, VariableExecutionLoadMaintainsPhase) {
  const std::size_t maximum_iterations = calibrate_iterations(VARIABLE_WORK_MAX_TIME);

  EXPECT_DOUBLE_EQ(work_result, INITIAL_A + INITIAL_B);

  const std::size_t minimum_iterations = maximum_iterations / 8;

  BPE_SimpleScheduler scheduler;

  RecordingTimerBlock block{minimum_iterations, maximum_iterations};

  ASSERT_EQ(scheduler.configure(normal_test_config()), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.add("timer", block, timer_config(PERIOD)), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.start(), SchedulerResult::SUCCESS);

  ASSERT_TRUE(wait_for_samples(
      block, SAMPLE_COUNT, std::chrono::duration_cast<std::chrono::milliseconds>(TEST_TIMEOUT)));

  ASSERT_EQ(scheduler.stop(), SchedulerResult::SUCCESS);

  check_periodic_timing(block);
  check_block_result(block);

  bool execution_time_changes = false;

  for (std::size_t i = 1; i < SAMPLE_COUNT; ++i) {
    if (block.iterations(i) != block.iterations(0)) {
      execution_time_changes = true;
    }
  }

  EXPECT_TRUE(execution_time_changes);
}

TEST(BPESimpleSchedulerTest, RecoversNominalPhaseAfterOverrun) {
  const std::size_t normal_iterations = calibrate_iterations(NORMAL_WORK_TIME);
  EXPECT_DOUBLE_EQ(work_result, INITIAL_A + INITIAL_B);

  const std::size_t overrun_iterations = calibrate_iterations(OVERRUN_WORK_TIME);
  EXPECT_DOUBLE_EQ(work_result, INITIAL_A + INITIAL_B);

  BPE_SimpleScheduler scheduler;

  RecordingTimerBlock block{normal_iterations, normal_iterations, overrun_iterations};

  ASSERT_EQ(scheduler.configure(normal_test_config()), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.add("timer", block, timer_config(PERIOD)), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.start(), SchedulerResult::SUCCESS);

  ASSERT_TRUE(wait_for_samples(block, 4, 1s));

  ASSERT_EQ(scheduler.stop(), SchedulerResult::SUCCESS);

  EXPECT_GT(block.execution_time(0), PERIOD);

  //
  // The first execution overruns the second nominal activation.
  // The third activation must still correspond to:
  //
  //   t0 + 2 * PERIOD
  //
  // rather than to:
  //
  //   previous_execution + PERIOD
  //
  const std::int64_t expected_third = block.timestamp_ns(0) + 2 * PERIOD_NS;

  const auto third_phase_error = std::chrono::nanoseconds{block.timestamp_ns(2) - expected_third};

  EXPECT_GE(third_phase_error, -PHASE_TOLERANCE);

  EXPECT_LE(third_phase_error, PHASE_TOLERANCE);

  check_block_result(block);
}

TEST(BPESimpleSchedulerTest, OverloadedBlocksRemainFair) {
  constexpr std::size_t EVENT_COUNT = 32;

  const std::size_t overload_iterations = calibrate_iterations(OVERLOAD_WORK_TIME);
  EXPECT_DOUBLE_EQ(work_result, INITIAL_A + INITIAL_B);

  ExecutionLog log;

  RecordingTimerBlock first{overload_iterations, overload_iterations, 0, RANDOM_SEED, &log, 1};

  RecordingTimerBlock second{overload_iterations, overload_iterations, 0, RANDOM_SEED + 1, &log, 2};

  BPE_SimpleScheduler scheduler;

  ASSERT_EQ(scheduler.configure(normal_test_config()), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.add("first", first, timer_config(PERIOD)), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.add("second", second, timer_config(PERIOD)), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.start(), SchedulerResult::SUCCESS);

  ASSERT_TRUE(wait_for_events(log, EVENT_COUNT, 3s));

  ASSERT_EQ(scheduler.stop(), SchedulerResult::SUCCESS);

  //
  // Verify that the test really overloaded the single scheduler thread.
  // The combined work required by both blocks must exceed one period.
  //
  ASSERT_GT(first.count(), 0U);
  ASSERT_GT(second.count(), 0U);

  const auto combined_execution_time = first.execution_time(0) + second.execution_time(0);

  ASSERT_GT(combined_execution_time, PERIOD);

  //
  // A permanently overdue block must not monopolize the worker.
  // Each scheduler pass executes each overdue block at most once.
  //
  const std::size_t recorded_events = std::min(EVENT_COUNT, log.count());

  for (std::size_t i = 1; i < recorded_events; ++i) {
    EXPECT_NE(log.event(i), log.event(i - 1))
        << "Block " << log.event(i) << " executed twice consecutively at event " << i;
  }

  const auto first_count = static_cast<std::int64_t>(first.count());

  const auto second_count = static_cast<std::int64_t>(second.count());

  EXPECT_LE(std::llabs(first_count - second_count), 1);

  check_block_result(first);
  check_block_result(second);
}

TEST(BPESimpleSchedulerTest, ScheduleIsImmutableWhileRunning) {
  const std::size_t work_iterations = calibrate_iterations(NORMAL_WORK_TIME);
  EXPECT_DOUBLE_EQ(work_result, INITIAL_A + INITIAL_B);

  BPE_SimpleScheduler scheduler;

  RecordingTimerBlock first{work_iterations, work_iterations};

  RecordingTimerBlock second{work_iterations, work_iterations};

  ASSERT_EQ(scheduler.configure(normal_test_config()), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.add("first", first, timer_config(PERIOD)), SchedulerResult::SUCCESS);

  ASSERT_EQ(scheduler.start(), SchedulerResult::SUCCESS);

  EXPECT_EQ(scheduler.add("second", second, timer_config(PERIOD)), SchedulerResult::INVALID_STATE);

  EXPECT_EQ(scheduler.remove("first"), SchedulerResult::INVALID_STATE);

  EXPECT_EQ(scheduler.clear(), SchedulerResult::INVALID_STATE);

  ASSERT_EQ(scheduler.stop(), SchedulerResult::SUCCESS);

  EXPECT_EQ(scheduler.remove("first"), SchedulerResult::SUCCESS);

  EXPECT_EQ(scheduler.clear(), SchedulerResult::SUCCESS);
}

TEST(BPESimpleSchedulerTest, RejectsInvalidConfiguration) {
  BPE_SimpleScheduler scheduler;

  const SchedulerConfig config = YAML::Load(R"(
realtime:
  enabled: false

timing:
  spin_before_deadline: nonsense
)");

  EXPECT_EQ(scheduler.configure(config), SchedulerResult::INVALID_CONFIG);
}

} // namespace
} // namespace bpe::test