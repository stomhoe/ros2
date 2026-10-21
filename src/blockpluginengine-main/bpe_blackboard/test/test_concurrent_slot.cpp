#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "bpe/lfbb/details/concurrent_slot.hpp"

namespace bpe::lfbb::details {
namespace {

struct TestProtectedData {
  std::int32_t value{0};
  std::int32_t marker{0};
};

using TestSlot = ConcurrentSlot<TestProtectedData>;

static_assert(!std::is_copy_constructible_v<TestSlot>);
static_assert(!std::is_copy_assignable_v<TestSlot>);
static_assert(!std::is_move_constructible_v<TestSlot>);
static_assert(!std::is_move_assignable_v<TestSlot>);

TEST(ConcurrentSlotTests, StartsFreeWithDefaultMetadataAndData) {
  TestSlot slot;

  EXPECT_EQ(slot.sequence(), 0u);
  EXPECT_EQ(slot.timestamp_ns(), 0u);
  EXPECT_EQ(slot.data().value, 0);
  EXPECT_EQ(slot.data().marker, 0);
  EXPECT_TRUE(slot.try_acquire_writer());

  slot.release_writer();
}

TEST(ConcurrentSlotTests, CanBeConstructedWithProtectedData) {
  TestSlot slot(TestProtectedData{42, 7});

  EXPECT_EQ(slot.data().value, 42);
  EXPECT_EQ(slot.data().marker, 7);
}

TEST(ConcurrentSlotTests, WriterReservationIsExclusive) {
  TestSlot slot;

  ASSERT_TRUE(slot.try_acquire_writer());
  EXPECT_FALSE(slot.try_acquire_writer());
  EXPECT_FALSE(slot.try_acquire_reader());

  slot.release_writer();

  EXPECT_TRUE(slot.try_acquire_writer());
  slot.release_writer();
}

TEST(ConcurrentSlotTests, SeveralReadersCanShareTheSlot) {
  TestSlot slot;

  ASSERT_TRUE(slot.try_acquire_reader());
  ASSERT_TRUE(slot.try_acquire_reader());
  ASSERT_TRUE(slot.try_acquire_reader());
  EXPECT_FALSE(slot.try_acquire_writer());

  slot.release_reader();
  EXPECT_FALSE(slot.try_acquire_writer());

  slot.release_reader();
  EXPECT_FALSE(slot.try_acquire_writer());

  slot.release_reader();
  EXPECT_TRUE(slot.try_acquire_writer());
  slot.release_writer();
}

TEST(ConcurrentSlotTests, WriterCanModifyProtectedData) {
  TestSlot slot;

  ASSERT_TRUE(slot.try_acquire_writer());
  slot.data().value = 123;
  slot.data().marker = 456;
  slot.release_writer();

  ASSERT_TRUE(slot.try_acquire_reader());
  const TestSlot& const_slot = slot;
  EXPECT_EQ(const_slot.data().value, 123);
  EXPECT_EQ(const_slot.data().marker, 456);
  slot.release_reader();
}

TEST(ConcurrentSlotTests, ConcurrentReadersCanAcquireTheSameSlot) {
  constexpr std::size_t reader_count = 8u;

  TestSlot slot(TestProtectedData{91, 17});
  std::atomic<std::size_t> acquired_count{0u};
  std::atomic<bool> release_readers{false};
  std::vector<std::thread> readers;
  readers.reserve(reader_count);

  for (std::size_t i = 0; i < reader_count; ++i) {
    readers.emplace_back([&slot, &acquired_count, &release_readers]() {
      const bool acquired = slot.try_acquire_reader();
      if (acquired) {
        acquired_count.fetch_add(1u, std::memory_order_release);
        while (!release_readers.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        slot.release_reader();
      }
    });
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (acquired_count.load(std::memory_order_acquire) != reader_count &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }

  const std::size_t final_acquired_count = acquired_count.load(std::memory_order_acquire);
  if (final_acquired_count == reader_count) {
    EXPECT_FALSE(slot.try_acquire_writer());
    EXPECT_EQ(slot.data().value, 91);
    EXPECT_EQ(slot.data().marker, 17);
  }

  release_readers.store(true, std::memory_order_release);

  for (auto& reader : readers) {
    reader.join();
  }

  ASSERT_EQ(final_acquired_count, reader_count);
  EXPECT_TRUE(slot.try_acquire_writer());
  slot.release_writer();
}

} // namespace
} // namespace bpe::lfbb::details
