#pragma once

#include <memory>

#include <rclcpp/node_options.hpp>

namespace bpe {

class BPE_LifecycleNode;
class BlockPluginManager;
class IBlockScheduler;

// Composition root for one complete BlockPluginEngine instance.
//
// The explicit ownership order is intentional.  Integration diagnostics showed
// that destroying BlockPluginManager/ClassLoader before BPE_LifecycleNode causes
// node teardown to fail when dynamically loaded plugins have created ROS
// subscriptions.  Keeping the manager alive also keeps the plugin DSOs loaded
// until the lifecycle node has completed its own destruction.
class BPECompositionRoot {
public:
  explicit BPECompositionRoot(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BPECompositionRoot();

  BPECompositionRoot(const BPECompositionRoot&) = delete;
  BPECompositionRoot& operator=(const BPECompositionRoot&) = delete;
  BPECompositionRoot(BPECompositionRoot&&) = delete;
  BPECompositionRoot& operator=(BPECompositionRoot&&) = delete;

  const std::shared_ptr<BPE_LifecycleNode>& node() const noexcept;

private:
  std::shared_ptr<IBlockScheduler> scheduler_;
  std::shared_ptr<BPE_LifecycleNode> node_;
  std::shared_ptr<BlockPluginManager> plugin_manager_;
};

} // namespace bpe
