#include <cstdint>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "bpe/lfbb/blackboard.hpp"

namespace bpe::lfbb {
namespace {

struct TestProtectedData {
  std::int32_t value{0};
  std::int32_t marker{0};
};

using TestBlackboard = Blackboard<TestProtectedData, 3u>;
using TestProducerGrant = ProducerGrant<TestProtectedData, 3u>;
using TestConsumerGrant = ConsumerGrant<TestProtectedData, 3u>;
using TestProducerFactory = ProducerGrantFactory<TestProtectedData, 3u>;
using TestConsumerFactory = ConsumerGrantFactory<TestProtectedData, 3u>;

static_assert(!std::is_copy_constructible_v<TestProducerGrant>);
static_assert(!std::is_copy_assignable_v<TestProducerGrant>);
static_assert(!std::is_move_constructible_v<TestProducerGrant>);
static_assert(!std::is_move_assignable_v<TestProducerGrant>);
static_assert(!std::is_copy_constructible_v<TestConsumerGrant>);
static_assert(!std::is_copy_assignable_v<TestConsumerGrant>);
static_assert(!std::is_move_constructible_v<TestConsumerGrant>);
static_assert(!std::is_move_assignable_v<TestConsumerGrant>);

static_assert(!std::is_copy_constructible_v<TestProducerFactory>);
static_assert(!std::is_copy_assignable_v<TestProducerFactory>);
static_assert(!std::is_move_constructible_v<TestProducerFactory>);
static_assert(!std::is_move_assignable_v<TestProducerFactory>);
static_assert(!std::is_copy_constructible_v<TestConsumerFactory>);
static_assert(!std::is_copy_assignable_v<TestConsumerFactory>);
static_assert(!std::is_move_constructible_v<TestConsumerFactory>);
static_assert(!std::is_move_assignable_v<TestConsumerFactory>);

static_assert(std::is_same_v<decltype(std::declval<TestProducerGrant&>().data()),
                             TestProtectedData&>);
static_assert(std::is_same_v<decltype(std::declval<const TestConsumerGrant&>().data()),
                             const TestProtectedData&>);

TEST(GrantTests, ProducerPublishesOnlyWhenGrantIsDestroyed) {
  TestBlackboard blackboard;
  auto producer = blackboard.register_producer("sample");
  auto consumer = blackboard.register_consumer("sample");

  {
    auto writer = producer.write_access();
    writer.data().value = 42;
    writer.data().marker = 7;

    auto reader = consumer.read_access();
    EXPECT_EQ(reader.status(), ReadStatus::no_data_yet);
    EXPECT_EQ(reader.sequence(), 0u);
    EXPECT_EQ(reader.data().value, 0);
  }

  auto reader = consumer.read_access();
  EXPECT_EQ(reader.status(), ReadStatus::ok);
  EXPECT_EQ(reader.sequence(), 1u);
  EXPECT_GT(reader.timestamp_ns(), 0u);
  EXPECT_EQ(reader.data().value, 42);
  EXPECT_EQ(reader.data().marker, 7);
}

TEST(GrantTests, ConsumerReportsNoUpdateAndMissedSamplesPerFactory) {
  TestBlackboard blackboard;
  auto producer = blackboard.register_producer("sample");
  auto consumer = blackboard.register_consumer("sample");

  {
    auto writer = producer.write_access();
    writer.data().value = 1;
  }

  {
    auto reader = consumer.read_access();
    EXPECT_EQ(reader.status(), ReadStatus::ok);
    EXPECT_TRUE(reader.has_new_value());
    EXPECT_FALSE(reader.has_missed());
    EXPECT_EQ(reader.missed(), 0u);
    EXPECT_EQ(reader.sequence(), 1u);
  }

  {
    auto reader = consumer.read_access();
    EXPECT_EQ(reader.status(), ReadStatus::no_update);
    EXPECT_FALSE(reader.has_new_value());
    EXPECT_FALSE(reader.has_missed());
    EXPECT_EQ(reader.missed(), 0u);
  }

  for (std::int32_t value = 2; value <= 4; ++value) {
    auto writer = producer.write_access();
    writer.data().value = value;
  }

  auto reader = consumer.read_access();
  EXPECT_EQ(reader.status(), ReadStatus::missed);
  EXPECT_TRUE(reader.has_new_value());
  EXPECT_TRUE(reader.has_missed());
  EXPECT_EQ(reader.missed(), 2u);
  EXPECT_EQ(reader.sequence(), 4u);
  EXPECT_EQ(reader.data().value, 4);
}

TEST(GrantTests, ConsumerReservationKeepsSelectedSampleStable) {
  TestBlackboard blackboard;
  auto producer = blackboard.register_producer("sample");
  auto consumer = blackboard.register_consumer("sample");

  {
    auto writer = producer.write_access();
    writer.data().value = 1;
  }

  {
    auto held_reader = consumer.read_access();
    ASSERT_EQ(held_reader.data().value, 1);
    ASSERT_EQ(held_reader.sequence(), 1u);

    for (std::int32_t value = 2; value <= 7; ++value) {
      auto writer = producer.write_access();
      writer.data().value = value;
    }

    EXPECT_EQ(held_reader.data().value, 1);
    EXPECT_EQ(held_reader.sequence(), 1u);
  }

  auto latest = consumer.read_access();
  EXPECT_EQ(latest.data().value, 7);
  EXPECT_EQ(latest.sequence(), 7u);
}

TEST(GrantTests, FactoryRejectsOverlappingGrantFromSameEndpoint) {
  TestBlackboard blackboard;
  auto producer = blackboard.register_producer("sample");
  auto consumer = blackboard.register_consumer("sample");

  {
    auto writer = producer.write_access();
    EXPECT_THROW(producer.write_access(), std::runtime_error);
  }

  EXPECT_NO_THROW({
    auto writer = producer.write_access();
    writer.data().value = 10;
  });

  {
    auto reader = consumer.read_access();
    EXPECT_THROW(consumer.read_access(), std::runtime_error);
  }

  EXPECT_NO_THROW({
    auto reader = consumer.read_access();
    EXPECT_EQ(reader.data().value, 10);
  });
}

} // namespace
} // namespace bpe::lfbb
