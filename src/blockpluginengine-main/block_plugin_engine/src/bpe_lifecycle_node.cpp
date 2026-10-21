#include "detail/bpe_lifecycle_node.hpp"

#include <string>

#include "block_plugin_engine/iblock_scheduler.hpp"
#include "detail/block_plugin_manager.hpp"
#include "detail/runtime_state.hpp"

namespace bpe {

BPE_LifecycleNode::BPE_LifecycleNode(
    const std::shared_ptr<IBlockScheduler>& scheduler,
    const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("block_plugin_engine", options),
    scheduler_(scheduler) {
  RCLCPP_DEBUG(get_logger(), "Constructing BPE_LifecycleNode");

  declare_parameter<std::string>("config_file", "");
  declare_parameter<bool>("autostart", false);

  RCLCPP_DEBUG(get_logger(), "BPE_LifecycleNode constructed");
}

void BPE_LifecycleNode::set_plugin_manager(
    const std::shared_ptr<BlockPluginManager>& plugin_manager) {
  plugin_manager_ = plugin_manager;
}

BPE_LifecycleNode::CallbackReturn BPE_LifecycleNode::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Configuring BlockPluginEngine");

  const auto config_file = get_parameter("config_file").as_string();
  if (config_file.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'config_file' cannot be empty");
    return CallbackReturn::FAILURE;
  }

  YAML::Node config;

  RCLCPP_DEBUG(get_logger(), "Loading configuration file '%s'", config_file.c_str());
  try {
    config = YAML::LoadFile(config_file);
  } catch (const YAML::Exception& error) {
    RCLCPP_ERROR(get_logger(),
                 "Could not load configuration file '%s': %s",
                 config_file.c_str(),
                 error.what());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(get_logger(), "Configuring BlockScheduler");
  if (scheduler_->configure(config["scheduler"]) != SchedulerResult::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Could not configure BlockScheduler");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_DEBUG(get_logger(), "BlockScheduler configured");

  RCLCPP_DEBUG(get_logger(), "Configuring BlockPlugins");
  if (!plugin_manager_->configure(config["blocks"])) {
    detail::reset_blackboard();
    detail::reset_ros_type_registry();
    RCLCPP_ERROR(get_logger(), "Could not configure BlockPlugins");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_DEBUG(get_logger(), "BlockPlugins configured");

  RCLCPP_INFO(get_logger(), "BlockPluginEngine configured");
  return CallbackReturn::SUCCESS;
}

BPE_LifecycleNode::CallbackReturn BPE_LifecycleNode::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Activating BlockPluginEngine");

  RCLCPP_DEBUG(get_logger(), "Starting BlockPlugins");
  if (!plugin_manager_->start()) {
    RCLCPP_ERROR(get_logger(), "Could not start BlockPlugins");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_DEBUG(get_logger(), "BlockPlugins started");

  RCLCPP_DEBUG(get_logger(), "Starting BlockScheduler");
  if (scheduler_->start() != SchedulerResult::SUCCESS) {
    plugin_manager_->stop();
    RCLCPP_ERROR(get_logger(), "Could not start BlockScheduler");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_DEBUG(get_logger(), "BlockScheduler started");

  RCLCPP_INFO(get_logger(), "BlockPluginEngine active");
  return CallbackReturn::SUCCESS;
}

BPE_LifecycleNode::CallbackReturn BPE_LifecycleNode::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Deactivating BlockPluginEngine");

  RCLCPP_DEBUG(get_logger(), "Stopping BlockScheduler");
  scheduler_->stop();
  RCLCPP_DEBUG(get_logger(), "BlockScheduler stopped");

  RCLCPP_DEBUG(get_logger(), "Stopping BlockPlugins");
  if (!plugin_manager_->stop()) {
    RCLCPP_ERROR(get_logger(), "Could not stop BlockPlugins");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_DEBUG(get_logger(), "BlockPlugins stopped");

  RCLCPP_INFO(get_logger(), "BlockPluginEngine inactive");
  return CallbackReturn::SUCCESS;
}

BPE_LifecycleNode::CallbackReturn BPE_LifecycleNode::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Cleaning up BlockPluginEngine");

  RCLCPP_DEBUG(get_logger(), "Stopping BlockScheduler before cleanup");
  scheduler_->stop();

  RCLCPP_DEBUG(get_logger(), "Cleaning up BlockPlugins");
  const bool cleanup_ok = plugin_manager_->cleanup();

  RCLCPP_DEBUG(get_logger(), "Resetting shared runtime state");
  detail::reset_blackboard();
  detail::reset_ros_type_registry();

  if (!cleanup_ok) {
    RCLCPP_ERROR(get_logger(), "Could not clean up BlockPlugins");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "BlockPluginEngine cleaned up");
  return CallbackReturn::SUCCESS;
}

BPE_LifecycleNode::CallbackReturn BPE_LifecycleNode::on_shutdown(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Shutting down BlockPluginEngine");

  RCLCPP_DEBUG(get_logger(), "Shutdown: stopping scheduler");
  scheduler_->stop();
  RCLCPP_DEBUG(get_logger(), "Shutdown: scheduler stopped");

  RCLCPP_DEBUG(get_logger(), "Shutdown: stopping plugins");
  const bool stop_ok = plugin_manager_->stop();
  RCLCPP_DEBUG(get_logger(), "Shutdown: plugins stopped (%s)", stop_ok ? "ok" : "with errors");

  RCLCPP_DEBUG(get_logger(), "Shutdown: cleaning up plugins");
  const bool cleanup_ok = plugin_manager_->cleanup();
  RCLCPP_DEBUG(get_logger(),
               "Shutdown: plugins cleaned up (%s)",
               cleanup_ok ? "ok" : "with errors");

  RCLCPP_DEBUG(get_logger(), "Shutdown: resetting blackboard");
  detail::reset_blackboard();
  RCLCPP_DEBUG(get_logger(), "Shutdown: blackboard reset");

  RCLCPP_DEBUG(get_logger(), "Shutdown: resetting ROS type registry");
  detail::reset_ros_type_registry();
  RCLCPP_DEBUG(get_logger(), "Shutdown: ROS type registry reset");

  RCLCPP_INFO(get_logger(), "BlockPluginEngine shutdown completed");
  return CallbackReturn::SUCCESS;
}

} // namespace bpe
