#include <cstdint>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "block_plugin_engine/typed_blackboard.hpp"

namespace bpe_system_test {

class TwistProducerBlock final
  : public bpe::IBlockPlugin,
    public bpe::ITimerBlock {
public:
  using Message = geometry_msgs::msg::TwistStamped;

  bpe::BlockResult configure(const bpe::BlockConfig& config) override {
    bpe::BlockResult result = bpe::BlockResult::SUCCESS;

    try {
      if (!config["blackboard_output"]) {
        result = bpe::BlockResult::FAILURE;
      } else {
        blackboard_output_ = config["blackboard_output"].as<std::string>();
        producer_.bind(blackboard_output_);
      }
    } catch (...) {
      result = bpe::BlockResult::FAILURE;
    }

    return result;
  }

  bpe::ExecutionResult execute() override {
    bpe::ExecutionResult result = bpe::ExecutionResult::SUCCESS;

    auto output = producer_.write_access();
    auto& message = output.value();

    ++sample_index_;
    const double value = static_cast<double>(sample_index_);

    message.header.frame_id = "bpe_system_test";
    message.twist.linear.x = value;
    message.twist.linear.y = 2.0 * value;
    message.twist.linear.z = 3.0 * value;
    message.twist.angular.x = -value;
    message.twist.angular.y = -2.0 * value;
    message.twist.angular.z = 0.5 * value;

    return result;
  }

private:
  bpe::TypedProducer<Message> producer_;
  std::string blackboard_output_;
  std::uint64_t sample_index_{0};
};

} // namespace bpe_system_test

PLUGINLIB_EXPORT_CLASS(
  bpe_system_test::TwistProducerBlock,
  bpe::IBlockPlugin)
