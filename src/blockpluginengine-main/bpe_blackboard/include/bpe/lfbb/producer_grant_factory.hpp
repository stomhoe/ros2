#pragma once

#include <cstddef>

#include "bpe/lfbb/producer_grant.hpp"

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin> class Blackboard;

namespace details {
template <typename ProtectedData, std::size_t InitializationMargin> class ConcurrentRing;
} // namespace details

template <typename ProtectedData, std::size_t InitializationMargin> class ProducerGrantFactory {
public:
  ~ProducerGrantFactory() = default;

  ProducerGrantFactory(const ProducerGrantFactory&) = delete;
  ProducerGrantFactory& operator=(const ProducerGrantFactory&) = delete;
  ProducerGrantFactory(ProducerGrantFactory&&) = delete;
  ProducerGrantFactory& operator=(ProducerGrantFactory&&) = delete;

  ProducerGrant<ProtectedData, InitializationMargin> write_access();

private:
  using Ring = details::ConcurrentRing<ProtectedData, InitializationMargin>;

  explicit ProducerGrantFactory(Ring* ring) noexcept;

  Ring* ring_{nullptr};
  bool active_{false};

  friend class Blackboard<ProtectedData, InitializationMargin>;
};

} // namespace bpe::lfbb

#include "bpe/lfbb/details/producer_grant_factory_impl.hpp"
