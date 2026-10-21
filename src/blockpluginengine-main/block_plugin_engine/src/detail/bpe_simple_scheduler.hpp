#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

#include "block_plugin_engine/iblock_scheduler.hpp"

namespace bpe {

struct SimpleSchedulerConfig {
  bool realtime_enabled{true};
  bool realtime_required{false};
  int realtime_priority{50};
  std::chrono::nanoseconds spin_before_deadline{std::chrono::microseconds{100}};
};

class BPE_SimpleScheduler final : public IBlockScheduler {
public:
  BPE_SimpleScheduler() = default;
  ~BPE_SimpleScheduler() override;

  SchedulerResult configure(const SchedulerConfig& config) override;

  SchedulerResult add(const std::string& instance_name,
                      ITimerBlock& block,
                      const ExecutionConfig& config) override;

  SchedulerResult remove(const std::string& instance_name) override;
  SchedulerResult start() override;
  SchedulerResult stop() override;
  SchedulerResult clear() override;

private:
  struct ScheduledBlock {
    std::string name;
    ITimerBlock* block{nullptr};
    std::int64_t period_ns{0};
    timespec next_execution{};
  };

  static bool time_before(const timespec& lhs, const timespec& rhs);
  static bool time_reached(const timespec& now, const timespec& deadline);
  static timespec subtract_ns(const timespec& time, std::int64_t nanoseconds);
  static void add_ns(timespec& time, std::int64_t nanoseconds);
  static bool parse_duration(const YAML::Node& node, std::chrono::nanoseconds& value);

  SchedulerResult validate_start_configuration() const;
  bool configure_realtime_thread();
  void initialize_deadlines();
  void worker_loop();
  void wait_until(const timespec& deadline);
  void busy_wait_until(const timespec& deadline);

  SimpleSchedulerConfig config_;
  std::vector<ScheduledBlock> blocks_;
  std::thread worker_;

  std::atomic<bool> running_{false};
  std::atomic<bool> worker_ready_{false};
  std::atomic<bool> start_gate_{false};
  std::atomic<bool> realtime_setup_ok_{false};
};

} // namespace bpe
