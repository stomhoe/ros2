#pragma once

#include <string>

#include "block_plugin_engine/execution_config.hpp"
#include "block_plugin_engine/scheduler_config.hpp"

namespace bpe {

class ITimerBlock;

enum class SchedulerResult { SUCCESS, INVALID_STATE, INVALID_CONFIG, NOT_FOUND, SYSTEM_ERROR };

class IBlockScheduler {
public:
  virtual ~IBlockScheduler() = default;

  virtual SchedulerResult configure(const SchedulerConfig& config) = 0;

  virtual SchedulerResult
  add(const std::string& instance_name, ITimerBlock& block, const ExecutionConfig& config) = 0;

  virtual SchedulerResult remove(const std::string& instance_name) = 0;
  virtual SchedulerResult start() = 0;
  virtual SchedulerResult stop() = 0;
  virtual SchedulerResult clear() = 0;
};

} // namespace bpe
