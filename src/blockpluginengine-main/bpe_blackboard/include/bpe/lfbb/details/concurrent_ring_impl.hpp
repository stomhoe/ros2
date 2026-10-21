#pragma once

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace bpe::lfbb::details {

template <typename ProtectedData, std::size_t InitializationMargin>
ConcurrentRing<ProtectedData, InitializationMargin>::ConcurrentRing() {
  resize_and_fill(compute_capacity(0u, 0u));
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ConcurrentRing<ProtectedData, InitializationMargin>::register_producer() {
  if (producer_registered_) {
    throw std::runtime_error("Producer already registered");
  } else if (!can_register_without_initializer()) {
    throw std::runtime_error("Initialization margin exhausted");
  } else {
    producer_registered_ = true;
    account_registration_without_initializer();
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
template <typename Initializer>
void ConcurrentRing<ProtectedData, InitializationMargin>::register_producer(
    const Initializer& initializer) {
  if (producer_registered_) {
    throw std::runtime_error("Producer already registered");
  } else {
    const std::size_t required_capacity = compute_capacity(consumer_count_, InitializationMargin);

    if (initialized_) {
      resize_and_fill(required_capacity, initializer);
    } else {
      resize_and_fill(required_capacity);
      initialize_all_slots(initializer);
      initialized_ = true;
    }

    producer_registered_ = true;
    reset_initialization_margin();
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ConcurrentRing<ProtectedData, InitializationMargin>::register_consumer() {
  if (!can_register_without_initializer()) {
    throw std::runtime_error("Initialization margin exhausted");
  } else {
    const std::size_t new_consumer_count = consumer_count_ + 1u;

    if (!initialized_) {
      const std::size_t required_capacity = compute_capacity(new_consumer_count, 0u);
      resize_and_fill(required_capacity);
    }

    consumer_count_ = new_consumer_count;
    account_registration_without_initializer();
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
template <typename Initializer>
void ConcurrentRing<ProtectedData, InitializationMargin>::register_consumer(
    const Initializer& initializer) {
  const std::size_t new_consumer_count = consumer_count_ + 1u;
  const std::size_t required_capacity = compute_capacity(new_consumer_count, InitializationMargin);

  if (initialized_) {
    resize_and_fill(required_capacity, initializer);
  } else {
    resize_and_fill(required_capacity);
    initialize_all_slots(initializer);
    initialized_ = true;
  }

  consumer_count_ = new_consumer_count;
  reset_initialization_margin();
}

template <typename ProtectedData, std::size_t InitializationMargin>
typename ConcurrentRing<ProtectedData, InitializationMargin>::Slot*
ConcurrentRing<ProtectedData, InitializationMargin>::claim_next_write_slot() noexcept {
  std::size_t sequence = last_committed_index_.load(std::memory_order_acquire) + 1u;
  Slot* claimed_slot = nullptr;

  while (claimed_slot == nullptr) {
    Slot* slot = buffer_[sequence & index_mask_].get();
    if (slot->try_acquire_writer()) {
      write_claim_index_ = sequence;
      claimed_slot = slot;
    } else {
      ++sequence;
    }
  }

  return claimed_slot;
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ConcurrentRing<ProtectedData, InitializationMargin>::commit_written_slot() noexcept {
  Slot* slot = buffer_[write_claim_index_ & index_mask_].get();

  ++commit_sequence_;
  slot->set_sequence(commit_sequence_);
  slot->set_timestamp_ns(now_ns());

  last_committed_index_.store(write_claim_index_, std::memory_order_release);
  slot->release_writer();
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ConcurrentRing<ProtectedData, InitializationMargin>::cancel_written_slot() noexcept {
  Slot* slot = buffer_[write_claim_index_ & index_mask_].get();
  slot->release_writer();
  write_claim_index_ = last_committed_index_.load(std::memory_order_relaxed);
}

template <typename ProtectedData, std::size_t InitializationMargin>
typename ConcurrentRing<ProtectedData, InitializationMargin>::Slot*
ConcurrentRing<ProtectedData, InitializationMargin>::get_last_committed_slot() noexcept {
  const std::size_t sequence = last_committed_index_.load(std::memory_order_acquire);
  return buffer_[sequence & index_mask_].get();
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::size_t ConcurrentRing<ProtectedData, InitializationMargin>::capacity() const noexcept {
  return capacity_;
}

template <typename ProtectedData, std::size_t InitializationMargin>
bool ConcurrentRing<ProtectedData, InitializationMargin>::has_producer() const noexcept {
  return producer_registered_;
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ConcurrentRing<ProtectedData, InitializationMargin>::resize_and_fill(
    std::size_t new_capacity) {
  if (new_capacity > capacity_) {
    std::vector<std::unique_ptr<Slot>> new_slots;
    new_slots.reserve(new_capacity - capacity_);

    for (std::size_t index = capacity_; index < new_capacity; ++index) {
      new_slots.emplace_back(std::make_unique<Slot>());
    }

    buffer_.reserve(new_capacity);
    for (auto& slot : new_slots) {
      buffer_.emplace_back(std::move(slot));
    }

    capacity_ = new_capacity;
    index_mask_ = new_capacity - 1u;
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
template <typename Initializer>
void ConcurrentRing<ProtectedData, InitializationMargin>::resize_and_fill(
    std::size_t new_capacity, const Initializer& initializer) {
  if (new_capacity > capacity_) {
    std::vector<std::unique_ptr<Slot>> new_slots;
    new_slots.reserve(new_capacity - capacity_);

    for (std::size_t index = capacity_; index < new_capacity; ++index) {
      auto slot = std::make_unique<Slot>();
      initializer.initialize(slot->data());
      new_slots.emplace_back(std::move(slot));
    }

    buffer_.reserve(new_capacity);
    for (auto& slot : new_slots) {
      buffer_.emplace_back(std::move(slot));
    }

    capacity_ = new_capacity;
    index_mask_ = new_capacity - 1u;
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
template <typename Initializer>
void ConcurrentRing<ProtectedData, InitializationMargin>::initialize_all_slots(
    const Initializer& initializer) {
  for (auto& slot : buffer_) {
    initializer.initialize(slot->data());
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
bool ConcurrentRing<ProtectedData, InitializationMargin>::can_register_without_initializer()
    const noexcept {
  return !initialized_ || registrations_since_initialization_ < InitializationMargin;
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ConcurrentRing<ProtectedData,
                    InitializationMargin>::account_registration_without_initializer() {
  if (initialized_) {
    ++registrations_since_initialization_;
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ConcurrentRing<ProtectedData, InitializationMargin>::reset_initialization_margin() noexcept {
  registrations_since_initialization_ = 0u;
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::uint64_t ConcurrentRing<ProtectedData, InitializationMargin>::now_ns() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::size_t
ConcurrentRing<ProtectedData, InitializationMargin>::next_power_of_two(std::size_t value) noexcept {
  std::size_t power = 1u;
  if (value > 1u) {
    while (power < value) {
      power <<= 1u;
    }
  }
  return power;
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::size_t ConcurrentRing<ProtectedData, InitializationMargin>::compute_capacity(
    std::size_t consumer_count, std::size_t uninitialized_registration_margin) noexcept {
  const std::size_t base_capacity =
      std::max(kMinimumCapacity, consumer_count + kWaitFreeSlotMargin);
  return next_power_of_two(base_capacity + uninitialized_registration_margin);
}

} // namespace bpe::lfbb::details
