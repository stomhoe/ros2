
#include <cmath>
#include <exception>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "block_plugin_engine/typed_blackboard.hpp"

namespace my_cpp_package
{

class TwistScaleBlock final : public bpe::IBlockPlugin, public bpe::ITimerBlock
{
public:
  using Message = geometry_msgs::msg::TwistStamped;

  bpe::BlockResult configure(const bpe::BlockConfig & config) override
  {
    try {
      if (!config["blackboard_input"] || !config["blackboard_output"]) {
        return bpe::BlockResult::FAILURE;
      }

      blackboard_input_ = config["blackboard_input"].as<std::string>();
      blackboard_output_ = config["blackboard_output"].as<std::string>();
      scale_ = config["scale"].as<double>(2.0);
      if (blackboard_input_.empty() || blackboard_output_.empty() ||
        !std::isfinite(scale_))
      {
        return bpe::BlockResult::FAILURE;
      }

      consumer_.bind(blackboard_input_);
      producer_.bind(blackboard_output_);
      return bpe::BlockResult::SUCCESS;
    } catch (const std::exception &) {
      return bpe::BlockResult::FAILURE;
    }
  }

  bpe::ExecutionResult execute() override
  {
    auto input = consumer_.read_access();
    if (input.has_new_value()) {
      auto output = producer_.write_access();
      output.value() = input.value();
      output.value().twist.linear.x *= scale_;

    }

    return bpe::ExecutionResult::SUCCESS;
  }

private:
  bpe::TypedConsumer<Message> consumer_;
  bpe::TypedProducer<Message> producer_;
  std::string blackboard_input_;
  std::string blackboard_output_;
  double scale_{2.0};
};

}

PLUGINLIB_EXPORT_CLASS(
  my_cpp_package::TwistScaleBlock,
  bpe::IBlockPlugin)
