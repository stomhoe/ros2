#pragma once

#include "bpe/lfbb/details/concurrent_ring.hpp"

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin>
ConsumerGrant<ProtectedData, InitializationMargin>::ConsumerGrant(
    Ring* ring,
    bool& active,
    std::uint64_t& last_seen_sequence) noexcept
    : active_(active) {
  do {
    slot_ = ring->get_last_committed_slot();
  } while (!slot_->try_acquire_reader());

  const std::uint64_t current_sequence = slot_->sequence();
  if (current_sequence == 0u) {
    status_ = ReadStatus::no_data_yet;
  } else if (current_sequence == last_seen_sequence) {
    status_ = ReadStatus::no_update;
  } else if (current_sequence == last_seen_sequence + 1u) {
    status_ = ReadStatus::ok;
    last_seen_sequence = current_sequence;
  } else {
    status_ = ReadStatus::missed;
    missed_ = current_sequence - last_seen_sequence - 1u;
    last_seen_sequence = current_sequence;
  }
}

template <typename ProtectedData, std::size_t InitializationMargin>
ConsumerGrant<ProtectedData, InitializationMargin>::~ConsumerGrant() {
  slot_->release_reader();
  active_ = false;
}

template <typename ProtectedData, std::size_t InitializationMargin>
const ProtectedData& ConsumerGrant<ProtectedData, InitializationMargin>::data() const noexcept {
  return slot_->data();
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::uint64_t ConsumerGrant<ProtectedData, InitializationMargin>::sequence() const noexcept {
  return slot_->sequence();
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::uint64_t ConsumerGrant<ProtectedData, InitializationMargin>::timestamp_ns() const noexcept {
  return slot_->timestamp_ns();
}

template <typename ProtectedData, std::size_t InitializationMargin>
ReadStatus ConsumerGrant<ProtectedData, InitializationMargin>::status() const noexcept {
  return status_;
}

template <typename ProtectedData, std::size_t InitializationMargin>
bool ConsumerGrant<ProtectedData, InitializationMargin>::has_new_value() const noexcept {
  return status_ == ReadStatus::ok || status_ == ReadStatus::missed;
}

template <typename ProtectedData, std::size_t InitializationMargin>
bool ConsumerGrant<ProtectedData, InitializationMargin>::has_missed() const noexcept {
  return status_ == ReadStatus::missed;
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::uint64_t ConsumerGrant<ProtectedData, InitializationMargin>::missed() const noexcept {
  return missed_;
}

} // namespace bpe::lfbb
