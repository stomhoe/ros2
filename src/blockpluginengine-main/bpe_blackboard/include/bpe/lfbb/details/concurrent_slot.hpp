#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace bpe::lfbb::details {

template <typename ProtectedData, std::size_t InitializationMargin> class ConcurrentRing;

template <typename ProtectedData> class ConcurrentSlot {
public:
  ConcurrentSlot() = default;
  explicit ConcurrentSlot(ProtectedData protected_data);
  ~ConcurrentSlot() = default;

  ConcurrentSlot(const ConcurrentSlot&) = delete;
  ConcurrentSlot& operator=(const ConcurrentSlot&) = delete;
  ConcurrentSlot(ConcurrentSlot&&) = delete;
  ConcurrentSlot& operator=(ConcurrentSlot&&) = delete;

  bool try_acquire_writer() noexcept;
  void release_writer() noexcept;

  bool try_acquire_reader() noexcept;
  void release_reader() noexcept;

  ProtectedData& data() noexcept;
  const ProtectedData& data() const noexcept;

  std::uint64_t sequence() const noexcept;
  std::uint64_t timestamp_ns() const noexcept;

private:
  void set_sequence(std::uint64_t sequence) noexcept;
  void set_timestamp_ns(std::uint64_t timestamp_ns) noexcept;

  static constexpr std::size_t WRITE_LOCKED = std::numeric_limits<std::size_t>::max();
  static constexpr std::size_t FREE = 0u;

  std::atomic<std::size_t> state_{FREE};
  std::uint64_t sequence_{0};
  std::uint64_t timestamp_ns_{0};
  ProtectedData data_{};

  template <typename, std::size_t> friend class ConcurrentRing;
};

} // namespace bpe::lfbb::details

#include "bpe/lfbb/details/concurrent_slot_impl.hpp"
