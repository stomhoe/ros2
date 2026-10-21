#include "detail/bpe_composition_root.hpp"

#include <memory>

#include <rclcpp/logging.hpp>

#include "detail/block_plugin_manager.hpp"
#include "detail/bpe_lifecycle_node.hpp"
#include "detail/bpe_simple_scheduler.hpp"

namespace bpe {

BPECompositionRoot::BPECompositionRoot(const rclcpp::NodeOptions& options) {
  const auto logger = rclcpp::get_logger("block_plugin_engine");
  RCLCPP_INFO(logger, "Constructing BlockPluginEngine composition root");

  RCLCPP_DEBUG(logger, "Composition root: creating scheduler");
  scheduler_ = std::make_shared<BPE_SimpleScheduler>();

  RCLCPP_DEBUG(logger, "Composition root: creating lifecycle node");
  node_ = std::make_shared<BPE_LifecycleNode>(scheduler_, options);

  RCLCPP_DEBUG(logger, "Composition root: creating plugin manager");
  plugin_manager_ = std::make_shared<BlockPluginManager>(
      scheduler_, node_->get_node_topics_interface(), node_->get_logger());

  RCLCPP_DEBUG(logger, "Composition root: wiring plugin manager into lifecycle node");
  node_->set_plugin_manager(plugin_manager_);

  RCLCPP_INFO(logger, "BlockPluginEngine composition root constructed");
}

BPECompositionRoot::~BPECompositionRoot() {
  const auto logger = rclcpp::get_logger("block_plugin_engine");
  RCLCPP_INFO(logger, "Destroying BlockPluginEngine composition root");

  // This order is part of the runtime correctness contract.  Integration
  // diagnostics showed that subscriptions created by dynamically loaded plugins
  // require the plugin manager/ClassLoader to remain alive through LifecycleNode
  // destruction.  Do not replace this with incidental member destruction order.
  RCLCPP_DEBUG(logger, "Composition root: destroying lifecycle node");
  node_.reset();
  RCLCPP_DEBUG(logger, "Composition root: lifecycle node destroyed");

  RCLCPP_DEBUG(logger, "Composition root: destroying plugin manager");
  plugin_manager_.reset();
  RCLCPP_DEBUG(logger, "Composition root: plugin manager destroyed");

  RCLCPP_DEBUG(logger, "Composition root: destroying scheduler");
  scheduler_.reset();
  RCLCPP_DEBUG(logger, "Composition root: scheduler destroyed");

  RCLCPP_INFO(logger, "BlockPluginEngine composition root destroyed");
}

const std::shared_ptr<BPE_LifecycleNode>& BPECompositionRoot::node() const noexcept {
  return node_;
}

} // namespace bpe
