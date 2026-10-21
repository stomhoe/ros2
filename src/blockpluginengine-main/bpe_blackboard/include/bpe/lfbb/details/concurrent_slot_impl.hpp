#pragma once

namespace bpe::lfbb::details {

template <typename ProtectedData>
ConcurrentSlot<ProtectedData>::ConcurrentSlot(ProtectedData protected_data)
    : data_(std::move(protected_data)) {}

template <typename ProtectedData>
bool ConcurrentSlot<ProtectedData>::try_acquire_writer() noexcept {
  std::size_t expected = FREE;
  return state_.compare_exchange_strong(
      expected, WRITE_LOCKED, std::memory_order_acq_rel, std::memory_order_acquire);
}

template <typename ProtectedData> void ConcurrentSlot<ProtectedData>::release_writer() noexcept {
  state_.store(FREE, std::memory_order_release);
}

template <typename ProtectedData>
bool ConcurrentSlot<ProtectedData>::try_acquire_reader() noexcept {
  std::size_t current = state_.load(std::memory_order_acquire);
  bool acquired = false;

  while (!acquired && current != WRITE_LOCKED) {
    acquired = state_.compare_exchange_weak(
        current, current + 1, std::memory_order_acq_rel, std::memory_order_acquire);
  }

  return acquired;
}

template <typename ProtectedData> void ConcurrentSlot<ProtectedData>::release_reader() noexcept {
  state_.fetch_sub(1, std::memory_order_release);
}

template <typename ProtectedData> ProtectedData& ConcurrentSlot<ProtectedData>::data() noexcept {
  return data_;
}

template <typename ProtectedData>
const ProtectedData& ConcurrentSlot<ProtectedData>::data() const noexcept {
  return data_;
}

template <typename ProtectedData>
std::uint64_t ConcurrentSlot<ProtectedData>::sequence() const noexcept {
  return sequence_;
}

template <typename ProtectedData>
std::uint64_t ConcurrentSlot<ProtectedData>::timestamp_ns() const noexcept {
  return timestamp_ns_;
}

template <typename ProtectedData>
void ConcurrentSlot<ProtectedData>::set_sequence(std::uint64_t sequence) noexcept {
  sequence_ = sequence;
}

template <typename ProtectedData>
void ConcurrentSlot<ProtectedData>::set_timestamp_ns(std::uint64_t timestamp_ns) noexcept {
  timestamp_ns_ = timestamp_ns;
}

} // namespace bpe::lfbb::details
