#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/serialized_message.hpp>

#include "bpe/lfbb/consumer_grant.hpp"
#include "block_plugin_engine/blackboard.hpp"
#include "block_plugin_engine/ros_type_registry.hpp"

namespace bpe_ros_boundary_plugins::detail {

using ProducerFactory = decltype(
    std::declval<bpe::BpeBlackboard&>().register_producer(std::declval<const std::string&>()));
using ConsumerFactory = decltype(
    std::declval<bpe::BpeBlackboard&>().register_consumer(std::declval<const std::string&>()));
using ProducerGrant = decltype(std::declval<ProducerFactory&>().write_access());
using ConsumerGrant = decltype(std::declval<ConsumerFactory&>().read_access());

class SerializedProducer;
class SerializedConsumer;

class SerializedProducerAdapter {
public:
  ~SerializedProducerAdapter() noexcept(false) {
    try {
      type_info_->deserialize(grant_.data().serialized, grant_.data().native.get());
    } catch (...) {
      grant_.cancel();
      throw;
    }
  }

  rclcpp::SerializedMessage& serialized() noexcept {
    return grant_.data().serialized;
  }

  SerializedProducerAdapter(const SerializedProducerAdapter&) = delete;
  SerializedProducerAdapter& operator=(const SerializedProducerAdapter&) = delete;
  SerializedProducerAdapter(SerializedProducerAdapter&&) = delete;
  SerializedProducerAdapter& operator=(SerializedProducerAdapter&&) = delete;

private:
  SerializedProducerAdapter(ProducerFactory& factory, const bpe::RosTypeInfo& type_info)
    : grant_(factory.write_access()), type_info_(&type_info) {}

  ProducerGrant grant_;
  const bpe::RosTypeInfo* type_info_{nullptr};

  friend class SerializedProducer;
};

class SerializedConsumerAdapter {
public:
  ~SerializedConsumerAdapter() = default;

  const rclcpp::SerializedMessage& serialized() const noexcept {
    return grant_.data().serialized;
  }

  std::uint64_t sequence() const noexcept {
    return grant_.sequence();
  }

  std::uint64_t timestamp_ns() const noexcept {
    return grant_.timestamp_ns();
  }

  bpe::lfbb::ReadStatus status() const noexcept {
    return grant_.status();
  }

  bool has_new_value() const noexcept {
    return grant_.has_new_value();
  }

  bool has_missed() const noexcept {
    return grant_.has_missed();
  }

  std::uint64_t missed() const noexcept {
    return grant_.missed();
  }

  SerializedConsumerAdapter(const SerializedConsumerAdapter&) = delete;
  SerializedConsumerAdapter& operator=(const SerializedConsumerAdapter&) = delete;
  SerializedConsumerAdapter(SerializedConsumerAdapter&&) = delete;
  SerializedConsumerAdapter& operator=(SerializedConsumerAdapter&&) = delete;

private:
  explicit SerializedConsumerAdapter(ConsumerFactory& factory) : grant_(factory.read_access()) {}

  ConsumerGrant grant_;

  friend class SerializedConsumer;
};

class SerializedProducer {
public:
  SerializedProducer() = default;
  ~SerializedProducer() = default;

  void bind(const std::string& label, const std::string& ros_type) {
    if (binding_.has_value()) {
      throw std::logic_error("SerializedProducer already bound");
    } else {
      binding_.emplace(label, ros_type);
    }
  }

  bool has_type_operations() const noexcept {
    return binding_.has_value() && binding_->type_info->has_type_operations();
  }

  SerializedProducerAdapter write_access() {
    if (!binding_.has_value()) {
      throw std::logic_error("SerializedProducer is not bound");
    }
    if (!binding_->type_info->has_type_operations()) {
      throw std::logic_error("SerializedProducer has no typed ROS operations");
    }

    return SerializedProducerAdapter(binding_->factory, *binding_->type_info);
  }

  SerializedProducer(const SerializedProducer&) = delete;
  SerializedProducer& operator=(const SerializedProducer&) = delete;
  SerializedProducer(SerializedProducer&&) = delete;
  SerializedProducer& operator=(SerializedProducer&&) = delete;

private:
  struct Binding {
    Binding(const std::string& label, const std::string& ros_type)
      : type_info(&bpe::the_ros_type_registry().register_ros_input(label, ros_type)),
        factory(bpe::the_blackboard().register_producer(label)) {}

    const bpe::RosTypeInfo* type_info;
    ProducerFactory factory;
  };

  std::optional<Binding> binding_;
};

class SerializedConsumer {
public:
  SerializedConsumer() = default;
  ~SerializedConsumer() = default;

  void bind(const std::string& label, const std::string& ros_type) {
    if (binding_.has_value()) {
      throw std::logic_error("SerializedConsumer already bound");
    } else {
      binding_.emplace(label, ros_type);
    }
  }

  SerializedConsumerAdapter read_access() {
    if (!binding_.has_value()) {
      throw std::logic_error("SerializedConsumer is not bound");
    }

    return SerializedConsumerAdapter(binding_->factory);
  }

  SerializedConsumer(const SerializedConsumer&) = delete;
  SerializedConsumer& operator=(const SerializedConsumer&) = delete;
  SerializedConsumer(SerializedConsumer&&) = delete;
  SerializedConsumer& operator=(SerializedConsumer&&) = delete;

private:
  struct Binding {
    Binding(const std::string& label, const std::string& ros_type)
      : type_info(&bpe::the_ros_type_registry().register_ros_output(label, ros_type)),
        factory(bpe::the_blackboard().register_consumer(label)) {}

    const bpe::RosTypeInfo* type_info;
    ConsumerFactory factory;
  };

  std::optional<Binding> binding_;
};

} // namespace bpe_ros_boundary_plugins::detail
