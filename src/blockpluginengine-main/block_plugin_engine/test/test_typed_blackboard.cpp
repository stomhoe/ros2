#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/vector3.hpp>

#include "block_plugin_engine/typed_blackboard.hpp"

namespace {

using TestMessage = geometry_msgs::msg::Vector3;

TEST(TypedBlackboard, ProducerAndConsumerShareTypedNativeRepresentation) {
  bpe::TypedProducer<TestMessage> producer;
  bpe::TypedConsumer<TestMessage> consumer;

  producer.bind("typed_blackboard.sample");
  consumer.bind("typed_blackboard.sample");

  {
    auto output = producer.write_access();
    output.value().x = 42.0;
    output.value().y = 1.5;
  }

  auto input = consumer.read_access();

  ASSERT_TRUE(input.has_new_value());
  EXPECT_DOUBLE_EQ(input.value().x, 42.0);
  EXPECT_DOUBLE_EQ(input.value().y, 1.5);
}

TEST(TypedBlackboard, ConsumerCanBindBeforeProducerWhenItKnowsTheType) {
  bpe::TypedConsumer<TestMessage> consumer;
  bpe::TypedProducer<TestMessage> producer;

  consumer.bind("typed_blackboard.consumer_first");
  producer.bind("typed_blackboard.consumer_first");

  {
    auto output = producer.write_access();
    output.value().x = 7.0;
  }

  auto input = consumer.read_access();

  ASSERT_TRUE(input.has_new_value());
  EXPECT_DOUBLE_EQ(input.value().x, 7.0);
}

TEST(TypedBlackboard, AccessBeforeBindIsRejected) {
  bpe::TypedProducer<TestMessage> producer;
  bpe::TypedConsumer<TestMessage> consumer;

  EXPECT_THROW(producer.write_access(), std::logic_error);
  EXPECT_THROW(consumer.read_access(), std::logic_error);
}

TEST(TypedBlackboard, RebindingIsRejected) {
  bpe::TypedProducer<TestMessage> producer;
  bpe::TypedConsumer<TestMessage> consumer;

  producer.bind("typed_blackboard.rebind_producer");
  consumer.bind("typed_blackboard.rebind_consumer");

  EXPECT_THROW(producer.bind("typed_blackboard.other_producer"), std::logic_error);
  EXPECT_THROW(consumer.bind("typed_blackboard.other_consumer"), std::logic_error);
}

} // namespace
