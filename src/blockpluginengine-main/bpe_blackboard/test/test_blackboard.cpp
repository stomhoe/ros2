#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "bpe/lfbb/blackboard.hpp"

namespace bpe::lfbb {
namespace {

struct TestProtectedData {
  std::int32_t value{0};
};

struct TestInitializer {
  std::size_t* call_count{nullptr};
  std::int32_t value{0};

  void initialize(TestProtectedData& data) const {
    ++(*call_count);
    data.value = value;
  }
};

using TestBlackboard = Blackboard<TestProtectedData, 3u>;

static_assert(!std::is_copy_constructible_v<TestBlackboard>);
static_assert(!std::is_copy_assignable_v<TestBlackboard>);
static_assert(!std::is_move_constructible_v<TestBlackboard>);
static_assert(!std::is_move_assignable_v<TestBlackboard>);

TEST(BlackboardTests, EmptyBlackboardHasNoMissingProducer) {
  TestBlackboard blackboard;

  EXPECT_TRUE(blackboard.all_have_producer());
  EXPECT_TRUE(blackboard.labels_without_producer().empty());
}

TEST(BlackboardTests, RegistrationTracksLabelsWithoutProducer) {
  TestBlackboard blackboard;

  auto consumer_b = blackboard.register_consumer("b");
  auto consumer_a = blackboard.register_consumer("a");
  (void)consumer_a;
  (void)consumer_b;

  EXPECT_FALSE(blackboard.all_have_producer());
  EXPECT_EQ(blackboard.labels_without_producer(), (std::vector<std::string>{"a", "b"}));

  auto producer_a = blackboard.register_producer("a");
  (void)producer_a;
  EXPECT_FALSE(blackboard.all_have_producer());
  EXPECT_EQ(blackboard.labels_without_producer(), (std::vector<std::string>{"b"}));

  auto producer_b = blackboard.register_producer("b");
  (void)producer_b;
  EXPECT_TRUE(blackboard.all_have_producer());
  EXPECT_TRUE(blackboard.labels_without_producer().empty());
}

TEST(BlackboardTests, RejectsSecondProducerForSameLabel) {
  TestBlackboard blackboard;

  auto producer = blackboard.register_producer("sample");
  (void)producer;

  EXPECT_THROW(blackboard.register_producer("sample"), std::runtime_error);
}

TEST(BlackboardTests, RegistrationWithInitializerPreparesProtectedData) {
  TestBlackboard blackboard;
  std::size_t call_count = 0u;
  const TestInitializer initializer{&call_count, 31};

  auto producer = blackboard.register_producer("sample", initializer);
  auto consumer = blackboard.register_consumer("sample");

  EXPECT_EQ(call_count, 8u);

  {
    auto reader = consumer.read_access();
    EXPECT_EQ(reader.status(), ReadStatus::no_data_yet);
    EXPECT_EQ(reader.data().value, 31);
  }

  {
    auto writer = producer.write_access();
    writer.data().value = 47;
  }

  auto reader = consumer.read_access();
  EXPECT_EQ(reader.status(), ReadStatus::ok);
  EXPECT_EQ(reader.data().value, 47);
}

TEST(BlackboardTests, LabelsUseIndependentRings) {
  TestBlackboard blackboard;
  auto producer_a = blackboard.register_producer("a");
  auto producer_b = blackboard.register_producer("b");
  auto consumer_a = blackboard.register_consumer("a");
  auto consumer_b = blackboard.register_consumer("b");

  {
    auto writer = producer_a.write_access();
    writer.data().value = 11;
  }
  {
    auto writer = producer_b.write_access();
    writer.data().value = 22;
  }

  auto reader_a = consumer_a.read_access();
  auto reader_b = consumer_b.read_access();

  EXPECT_EQ(reader_a.data().value, 11);
  EXPECT_EQ(reader_b.data().value, 22);
  EXPECT_EQ(reader_a.sequence(), 1u);
  EXPECT_EQ(reader_b.sequence(), 1u);
}

TEST(BlackboardTests, SeveralConsumerEndpointsCanHoldDifferentSamples) {
  TestBlackboard blackboard;
  auto producer = blackboard.register_producer("sample");
  auto consumer_1 = blackboard.register_consumer("sample");
  auto consumer_2 = blackboard.register_consumer("sample");
  auto consumer_3 = blackboard.register_consumer("sample");
  auto consumer_4 = blackboard.register_consumer("sample");
  auto consumer_5 = blackboard.register_consumer("sample");
  auto consumer_6 = blackboard.register_consumer("sample");

  {
    auto writer = producer.write_access();
    writer.data().value = 1;
  }
  auto reader_1 = consumer_1.read_access();

  {
    auto writer = producer.write_access();
    writer.data().value = 2;
  }
  auto reader_2 = consumer_2.read_access();

  {
    auto writer = producer.write_access();
    writer.data().value = 3;
  }
  auto reader_3 = consumer_3.read_access();

  {
    auto writer = producer.write_access();
    writer.data().value = 4;
  }
  auto reader_4 = consumer_4.read_access();

  {
    auto writer = producer.write_access();
    writer.data().value = 5;
  }
  auto reader_5 = consumer_5.read_access();

  {
    auto writer = producer.write_access();
    writer.data().value = 6;
  }
  auto reader_6 = consumer_6.read_access();

  {
    auto writer = producer.write_access();
    writer.data().value = 99;
  }

  EXPECT_EQ(reader_1.data().value, 1);
  EXPECT_EQ(reader_2.data().value, 2);
  EXPECT_EQ(reader_3.data().value, 3);
  EXPECT_EQ(reader_4.data().value, 4);
  EXPECT_EQ(reader_5.data().value, 5);
  EXPECT_EQ(reader_6.data().value, 6);
}


TEST(BlackboardTests, UsesDefaultInitializationMargin) {
  Blackboard<TestProtectedData> blackboard;

  auto producer = blackboard.register_producer("default_margin");
  auto consumer = blackboard.register_consumer("default_margin");

  {
    auto output = producer.write_access();
    output.data().value = 19;
  }

  auto input = consumer.read_access();
  ASSERT_TRUE(input.has_new_value());
  EXPECT_EQ(input.data().value, 19);
}

} // namespace
} // namespace bpe::lfbb
