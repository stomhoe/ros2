#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/iblock_plugin.hpp"
#include "test_event_log.hpp"

namespace bpe::test {

class TestBasicBlock : public IBlockPlugin {
public:
  BlockResult configure(const BlockConfig& config) override {
    if (!config["id"] || !config["value"]) {
      return BlockResult::FAILURE;
    }

    id_ = config["id"].as<std::string>();
    value_ = config["value"].as<int>();

    record_event(id_ + ".configure:" + std::to_string(value_));
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

private:
  std::string id_;
  int value_{0};
};

} // namespace bpe::test

PLUGINLIB_EXPORT_CLASS(bpe::test::TestBasicBlock, bpe::IBlockPlugin)