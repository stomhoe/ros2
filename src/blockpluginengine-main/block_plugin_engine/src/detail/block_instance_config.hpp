#pragma once

#include <optional>
#include <string>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/execution_config.hpp"

namespace bpe {

struct BlockInstanceConfig {
  std::string name;
  std::string type;
  BlockConfig config;
  std::optional<ExecutionConfig> execution;
};

} // namespace bpe
