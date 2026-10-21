#include "block_plugin_engine/ros_type_registry.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

#include "detail/runtime_state.hpp"

namespace bpe {
namespace {

std::unique_ptr<RosTypeRegistry>& ros_type_registry_instance() {
  static std::unique_ptr<RosTypeRegistry> instance = std::make_unique<RosTypeRegistry>();
  return instance;
}

} // namespace


RosTypeInfo::RosTypeInfo(std::string ros_type) : ros_type_(std::move(ros_type)) {}

const std::string& RosTypeInfo::ros_type() const noexcept {
  return ros_type_;
}

bool RosTypeInfo::has_type_operations() const noexcept {
  return serialize_ != nullptr && deserialize_ != nullptr;
}

bool RosTypeInfo::requires_serialization() const noexcept {
  return ros_output_registered_;
}

void RosTypeInfo::serialize(const void* native, rclcpp::SerializedMessage& serialized) const {
  if (serialize_ == nullptr) {
    throw std::logic_error("ROS serialize operation is not registered");
  }

  serialize_(native, &serialized);
}

void RosTypeInfo::deserialize(const rclcpp::SerializedMessage& serialized, void* native) const {
  if (deserialize_ == nullptr) {
    throw std::logic_error("ROS deserialize operation is not registered");
  }

  deserialize_(&serialized, native);
}

const RosTypeInfo& RosTypeRegistry::register_ros_input(const std::string& label,
                                                       const std::string& ros_type) {
  return get_or_create(label, ros_type);
}

const RosTypeInfo& RosTypeRegistry::register_ros_output(const std::string& label,
                                                        const std::string& ros_type) {
  RosTypeInfo& info = get_or_create(label, ros_type);

  if (info.ros_output_registered_) {
    throw std::runtime_error("ROS output already registered for blackboard label '" + label + "'");
  }

  info.ros_output_registered_ = true;
  return info;
}

const RosTypeInfo& RosTypeRegistry::register_typed(
    const std::string& label,
    const std::string& ros_type,
    RosTypeInfo::SerializeFunction serialize,
    RosTypeInfo::DeserializeFunction deserialize) {
  if (serialize == nullptr || deserialize == nullptr) {
    throw std::invalid_argument("ROS type operations cannot be null");
  }

  RosTypeInfo& info = get_or_create(label, ros_type);

  if (!info.has_type_operations()) {
    info.serialize_ = serialize;
    info.deserialize_ = deserialize;
  }

  return info;
}

RosTypeInfo& RosTypeRegistry::get_or_create(const std::string& label,
                                            const std::string& ros_type) {
  if (label.empty()) {
    throw std::invalid_argument("Blackboard label cannot be empty");
  }
  if (ros_type.empty()) {
    throw std::invalid_argument("ROS type cannot be empty");
  }

  auto it = entries_.find(label);
  if (it == entries_.end()) {
    std::unique_ptr<RosTypeInfo> info(new RosTypeInfo(ros_type));
    RosTypeInfo* raw = info.get();
    entries_.emplace(label, std::move(info));
    return *raw;
  }

  if (it->second->ros_type() != ros_type) {
    throw std::runtime_error("ROS type mismatch for blackboard label '" + label + "': '" +
                             it->second->ros_type() + "' != '" + ros_type + "'");
  }

  return *it->second;
}

RosTypeRegistry& the_ros_type_registry() {
  return *ros_type_registry_instance();
}

namespace detail {

void reset_ros_type_registry() {
  auto replacement = std::make_unique<RosTypeRegistry>();
  auto& instance = ros_type_registry_instance();

  RCLCPP_DEBUG(rclcpp::get_logger("block_plugin_engine"),
              "Runtime state: destroying ROS type registry instance");
  instance = std::move(replacement);
  RCLCPP_DEBUG(rclcpp::get_logger("block_plugin_engine"),
              "Runtime state: ROS type registry instance destroyed and recreated");
}

} // namespace detail
} // namespace bpe
