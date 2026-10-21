#include <cstddef>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/create_generic_subscription.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/iros_boundary_block.hpp"

namespace bpe_ros_boundary_plugins::debug {
namespace {

void discard_message(std::shared_ptr<rclcpp::SerializedMessage>) {}

} // namespace

class DebugRosSubscriptionFunctionBlock final
  : public bpe::IBlockPlugin,
    public bpe::IRosBoundaryBlock {
public:
  void set_ros_interface(
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics) override {
    topics_ = topics;
  }

  bpe::BlockResult configure(const bpe::BlockConfig& config) override {
    bpe::BlockResult result = bpe::BlockResult::SUCCESS;

    try {
      if (topics_ == nullptr || !config["topic"] || !config["ros_type"]) {
        result = bpe::BlockResult::FAILURE;
      } else {
        const std::string topic = config["topic"].as<std::string>();
        const std::string ros_type = config["ros_type"].as<std::string>();
        std::size_t qos_depth = 10u;

        if (config["qos_depth"]) {
          qos_depth = config["qos_depth"].as<std::size_t>();
        }

        if (topic.empty() || ros_type.empty() || qos_depth == 0u) {
          result = bpe::BlockResult::FAILURE;
        } else {
          subscription_ = rclcpp::create_generic_subscription(
              topics_, topic, ros_type, rclcpp::QoS(qos_depth), &discard_message);
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
    subscription_.reset();
    return bpe::BlockResult::SUCCESS;
  }

private:
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
};

} // namespace bpe_ros_boundary_plugins::debug

PLUGINLIB_EXPORT_CLASS(
  bpe_ros_boundary_plugins::debug::DebugRosSubscriptionFunctionBlock,
  bpe::IBlockPlugin)
