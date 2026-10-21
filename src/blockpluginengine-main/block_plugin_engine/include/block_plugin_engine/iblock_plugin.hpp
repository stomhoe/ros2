#pragma once

#include "block_plugin_engine/block_config.hpp"

namespace bpe {

enum class BlockResult { SUCCESS, FAILURE };

class IBlockPlugin {
public:
  virtual ~IBlockPlugin() = default;

  virtual BlockResult configure(const BlockConfig& config) = 0;

  virtual BlockResult start() {
    return BlockResult::SUCCESS;
  }

  virtual BlockResult stop() {
    return BlockResult::SUCCESS;
  }

  virtual BlockResult cleanup() {
    return BlockResult::SUCCESS;
  }
};

} // namespace bpe
