#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/serialized_message.hpp>

namespace bpe {

class RosTypeRegistry;

class RosTypeInfo {
public:
  using SerializeFunction = void (*)(const void*, rclcpp::SerializedMessage*);
  using DeserializeFunction = void (*)(const rclcpp::SerializedMessage*, void*);

  const std::string& ros_type() const noexcept;
  bool has_type_operations() const noexcept;
  bool requires_serialization() const noexcept;

  void serialize(const void* native, rclcpp::SerializedMessage& serialized) const;
  void deserialize(const rclcpp::SerializedMessage& serialized, void* native) const;

private:
  explicit RosTypeInfo(std::string ros_type);

  std::string ros_type_;
  SerializeFunction serialize_{nullptr};
  DeserializeFunction deserialize_{nullptr};
  bool ros_output_registered_{false};

  friend class RosTypeRegistry;
};

// Process-wide configuration registry for the ROS representation associated
// with each blackboard label. Entries are completed during plugin configure
// and remain immutable during execution.
class RosTypeRegistry {
public:
  const RosTypeInfo& register_ros_input(const std::string& label,
                                        const std::string& ros_type);

  const RosTypeInfo& register_ros_output(const std::string& label,
                                         const std::string& ros_type);

  const RosTypeInfo& register_typed(const std::string& label,
                                    const std::string& ros_type,
                                    RosTypeInfo::SerializeFunction serialize,
                                    RosTypeInfo::DeserializeFunction deserialize);

private:
  RosTypeInfo& get_or_create(const std::string& label, const std::string& ros_type);

  std::unordered_map<std::string, std::unique_ptr<RosTypeInfo>> entries_;
};

RosTypeRegistry& the_ros_type_registry();

} // namespace bpe
