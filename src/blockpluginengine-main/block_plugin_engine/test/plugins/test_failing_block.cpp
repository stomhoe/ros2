#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/iblock_plugin.hpp"
#include "test_event_log.hpp"

namespace bpe::test {

class TestFailingBlock : public IBlockPlugin {
public:
  BlockResult configure(const BlockConfig& config) override {
    if (!config["id"]) {
      return BlockResult::FAILURE;
    }

    id_ = config["id"].as<std::string>();

    if (config["fail_on"]) {
      fail_on_ = config["fail_on"].as<std::string>();
    }

    record_event(id_ + ".configure");

    if (fail_on_ == "configure") {
      return BlockResult::FAILURE;
    }

    return BlockResult::SUCCESS;
  }

  BlockResult start() override {
    record_event(id_ + ".start");

    if (fail_on_ == "start") {
      return BlockResult::FAILURE;
    }

    return BlockResult::SUCCESS;
  }

  BlockResult stop() override {
    record_event(id_ + ".stop");

    if (fail_on_ == "stop") {
      return BlockResult::FAILURE;
    }

    return BlockResult::SUCCESS;
  }

  BlockResult cleanup() override {
    record_event(id_ + ".cleanup");

    if (fail_on_ == "cleanup") {
      return BlockResult::FAILURE;
    }

    return BlockResult::SUCCESS;
  }

private:
  std::string id_;
  std::string fail_on_;
};

} // namespace bpe::test

PLUGINLIB_EXPORT_CLASS(bpe::test::TestFailingBlock, bpe::IBlockPlugin)