#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "test_event_log.hpp"

namespace bpe::test {

class TestTimerBlock : public IBlockPlugin, public ITimerBlock {
public:
  BlockResult configure(const BlockConfig& config) override {
    if (!config["id"] || !config["label"]) {
      return BlockResult::FAILURE;
    }

    id_ = config["id"].as<std::string>();
    label_ = config["label"].as<std::string>();

    record_event(id_ + ".configure:" + label_);
    return BlockResult::SUCCESS;
  }

  BlockResult start() override {
    record_event(id_ + ".start");
    return BlockResult::SUCCESS;
  }

  BlockResult stop() override {
    record_event(id_ + ".stop");
    return BlockResult::SUCCESS;
  }

  BlockResult cleanup() override {
    record_event(id_ + ".cleanup");
    return BlockResult::SUCCESS;
  }

  ExecutionResult execute() override {
    record_event(id_ + ".execute");
    return ExecutionResult::SUCCESS;
  }

private:
  std::string id_;
  std::string label_;
};

} // namespace bpe::test

PLUGINLIB_EXPORT_CLASS(bpe::test::TestTimerBlock, bpe::IBlockPlugin)