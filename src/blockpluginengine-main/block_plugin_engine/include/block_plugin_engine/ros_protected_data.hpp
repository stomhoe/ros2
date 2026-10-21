#pragma once

#include <memory>

#include <rclcpp/serialized_message.hpp>

namespace bpe {

// Physical storage used by the BPE blackboard for ROS 2 data.
//
// Plugins should not manipulate these members directly. Typed and boundary
// adapters provide the supported access paths. The native representation is
// initialized by typed registration. The serialized representation is kept
// coherent only when a ROS output requires it, or when it is the source of a
// generic ROS input.
struct RosProtectedData {
  RosProtectedData() = default;

  std::shared_ptr<void> native;
  rclcpp::SerializedMessage serialized;
};

} // namespace bpe
