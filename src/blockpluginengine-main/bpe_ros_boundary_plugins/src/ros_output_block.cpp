#include <cstddef>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/create_generic_publisher.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/qos.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/iros_boundary_block.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "detail/serialized_blackboard.hpp"

namespace bpe_ros_boundary_plugins {

class RosOutputBlock final
  : public bpe::IBlockPlugin,
    public bpe::IRosBoundaryBlock,
    public bpe::ITimerBlock {
public:
  void set_ros_interface(
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics) override {
    topics_ = topics;
  }

  bpe::BlockResult configure(const bpe::BlockConfig& config) override {
    bpe::BlockResult result = bpe::BlockResult::SUCCESS;

    try {
      if (topics_ == nullptr || !config["topic"] || !config["ros_type"] ||
          !config["blackboard_input"]) {
        result = bpe::BlockResult::FAILURE;
      } else {
        topic_ = config["topic"].as<std::string>();
        ros_type_ = config["ros_type"].as<std::string>();
        blackboard_input_ = config["blackboard_input"].as<std::string>();

        if (config["qos_depth"]) {
          qos_depth_ = config["qos_depth"].as<std::size_t>();
        }

        if (topic_.empty() || ros_type_.empty() || blackboard_input_.empty() || qos_depth_ == 0u) {
          result = bpe::BlockResult::FAILURE;
        } else {
          consumer_.bind(blackboard_input_, ros_type_);
          publisher_ = rclcpp::create_generic_publisher(
              topics_, topic_, ros_type_, rclcpp::QoS(qos_depth_));
        }
      }
    } catch (...) {
      result = bpe::BlockResult::FAILURE;
    }

    return result;
  }

  bpe::BlockResult cleanup() override {
    publisher_.reset();
    return bpe::BlockResult::SUCCESS;
  }

  bpe::ExecutionResult execute() override {
    bpe::ExecutionResult result = bpe::ExecutionResult::SUCCESS;

    try {
      auto input = consumer_.read_access();
      if (input.has_new_value()) {
        publisher_->publish(input.serialized());
      }
    } catch (...) {
      result = bpe::ExecutionResult::FAILURE;
    }

    return result;
  }

private:
  detail::SerializedConsumer consumer_;

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  rclcpp::GenericPublisher::SharedPtr publisher_;

  std::string topic_;
  std::string ros_type_;
  std::string blackboard_input_;
  std::size_t qos_depth_{10u};
};

} // namespace bpe_ros_boundary_plugins

PLUGINLIB_EXPORT_CLASS(
  bpe_ros_boundary_plugins::RosOutputBlock,
  bpe::IBlockPlugin)
