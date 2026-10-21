#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "bpe/lfbb/details/concurrent_slot.hpp"

namespace bpe::lfbb::details {

template <typename ProtectedData, std::size_t InitializationMargin> class ConcurrentRing {
public:
  using Slot = ConcurrentSlot<ProtectedData>;

  ConcurrentRing();
  ~ConcurrentRing() = default;

  ConcurrentRing(const ConcurrentRing&) = delete;
  ConcurrentRing& operator=(const ConcurrentRing&) = delete;
  ConcurrentRing(ConcurrentRing&&) = delete;
  ConcurrentRing& operator=(ConcurrentRing&&) = delete;

  void register_producer();

  template <typename Initializer> void register_producer(const Initializer& initializer);

  void register_consumer();

  template <typename Initializer> void register_consumer(const Initializer& initializer);

  Slot* claim_next_write_slot() noexcept;
  void commit_written_slot() noexcept;
  void cancel_written_slot() noexcept;
  Slot* get_last_committed_slot() noexcept;

  std::size_t capacity() const noexcept;
  bool has_producer() const noexcept;

private:
  static constexpr std::size_t kMinimumCapacity = 4u;
  static constexpr std::size_t kWaitFreeSlotMargin = 2u;

  void resize_and_fill(std::size_t new_capacity);

  template <typename Initializer>
  void resize_and_fill(std::size_t new_capacity, const Initializer& initializer);

  template <typename Initializer> void initialize_all_slots(const Initializer& initializer);

  bool can_register_without_initializer() const noexcept;
  void account_registration_without_initializer();
  void reset_initialization_margin() noexcept;

  static std::uint64_t now_ns() noexcept;
  static std::size_t next_power_of_two(std::size_t value) noexcept;
  static std::size_t compute_capacity(std::size_t consumer_count,
                                      std::size_t uninitialized_registration_margin) noexcept;

  std::vector<std::unique_ptr<Slot>> buffer_;
  std::size_t capacity_{0};
  std::size_t index_mask_{0};
  std::size_t consumer_count_{0};
  std::size_t registrations_since_initialization_{0};
  std::size_t write_claim_index_{0};
  std::atomic<std::size_t> last_committed_index_{0};
  std::uint64_t commit_sequence_{0};
  bool producer_registered_{false};
  bool initialized_{false};
};

} // namespace bpe::lfbb::details

#include "bpe/lfbb/details/concurrent_ring_impl.hpp"
