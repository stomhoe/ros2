#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "block_plugin_engine/typed_blackboard.hpp"

namespace bpe_system_test {

class TwistTransformBlock final : public bpe::IBlockPlugin, public bpe::ITimerBlock {
public:
  using Message = geometry_msgs::msg::TwistStamped;

  bpe::BlockResult configure(const bpe::BlockConfig& config) override {
    bpe::BlockResult result = bpe::BlockResult::SUCCESS;

    try {
      if (!config["blackboard_input"] || !config["blackboard_output"]) {
        result = bpe::BlockResult::FAILURE;
      } else {
        blackboard_input_ = config["blackboard_input"].as<std::string>();

        blackboard_output_ = config["blackboard_output"].as<std::string>();

        if (config["linear_x_scale"]) {
          linear_x_scale_ = config["linear_x_scale"].as<double>();
        }

        if (config["linear_y_scale"]) {
          linear_y_scale_ = config["linear_y_scale"].as<double>();
        }

        if (config["angular_z_offset"]) {
          angular_z_offset_ = config["angular_z_offset"].as<double>();
        }

        consumer_.bind(blackboard_input_);

        producer_.bind(blackboard_output_);
      }
    } catch (...) {
      result = bpe::BlockResult::FAILURE;
    }

    return result;
  }

  bpe::ExecutionResult execute() override {
    bpe::ExecutionResult result = bpe::ExecutionResult::SUCCESS;

    auto input = consumer_.read_access();

    if (input.has_new_value()) {
      auto output = producer_.write_access();

      output.value() = input.value();

      output.value().twist.linear.x *= linear_x_scale_;
      output.value().twist.linear.y *= linear_y_scale_;
      output.value().twist.angular.z += angular_z_offset_;
    }

    return result;
  }

private:
  bpe::TypedConsumer<Message> consumer_;
  bpe::TypedProducer<Message> producer_;

  std::string blackboard_input_;
  std::string blackboard_output_;

  double linear_x_scale_{1.0};
  double linear_y_scale_{1.0};
  double angular_z_offset_{0.0};
};

} // namespace bpe_system_test

PLUGINLIB_EXPORT_CLASS(bpe_system_test::TwistTransformBlock, bpe::IBlockPlugin)
