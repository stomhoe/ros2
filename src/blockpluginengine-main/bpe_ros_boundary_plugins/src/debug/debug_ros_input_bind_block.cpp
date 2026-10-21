#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "detail/serialized_blackboard.hpp"

namespace bpe_ros_boundary_plugins::debug {

class DebugRosInputBindBlock final : public bpe::IBlockPlugin {
public:
  bpe::BlockResult configure(const bpe::BlockConfig& config) override {
    bpe::BlockResult result = bpe::BlockResult::SUCCESS;

    try {
      if (!config["ros_type"] || !config["blackboard_output"]) {
        result = bpe::BlockResult::FAILURE;
      } else {
        const std::string ros_type = config["ros_type"].as<std::string>();
        const std::string blackboard_output = config["blackboard_output"].as<std::string>();

        if (ros_type.empty() || blackboard_output.empty()) {
          result = bpe::BlockResult::FAILURE;
        } else {
          producer_.bind(blackboard_output, ros_type);
        }
      }
    } catch (...) {
      result = bpe::BlockResult::FAILURE;
    }

    return result;
  }

  bpe::BlockResult start() override {
    return bpe::BlockResult::SUCCESS;
  }

  bpe::BlockResult stop() override {
    return bpe::BlockResult::SUCCESS;
  }

  bpe::BlockResult cleanup() override {
    return bpe::BlockResult::SUCCESS;
  }

private:
  detail::SerializedProducer producer_;
};

} // namespace bpe_ros_boundary_plugins::debug

PLUGINLIB_EXPORT_CLASS(
  bpe_ros_boundary_plugins::debug::DebugRosInputBindBlock,
  bpe::IBlockPlugin)
