#pragma once

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace bpe {

class BPECompositionRoot;
class IBlockScheduler;
class BlockPluginManager;

class BPE_LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit BPE_LifecycleNode(
      const std::shared_ptr<IBlockScheduler>& scheduler,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

private:
  friend class BPECompositionRoot;

  void set_plugin_manager(const std::shared_ptr<BlockPluginManager>& plugin_manager);

  std::shared_ptr<IBlockScheduler> scheduler_;
  std::shared_ptr<BlockPluginManager> plugin_manager_;
};

} // namespace bpe
