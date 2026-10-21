#pragma once

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin>
ProducerGrantFactory<ProtectedData, InitializationMargin>
Blackboard<ProtectedData, InitializationMargin>::register_producer(const std::string& label) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto [iterator, inserted] = entries_.try_emplace(label);
  (void)inserted;
  iterator->second.ring.register_producer();

  return ProducerGrantFactory<ProtectedData, InitializationMargin>(&(iterator->second.ring));
}

template <typename ProtectedData, std::size_t InitializationMargin>
template <typename Initializer>
ProducerGrantFactory<ProtectedData, InitializationMargin>
Blackboard<ProtectedData, InitializationMargin>::register_producer(
    const std::string& label,
    const Initializer& initializer) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto [iterator, inserted] = entries_.try_emplace(label);
  (void)inserted;
  iterator->second.ring.register_producer(initializer);

  return ProducerGrantFactory<ProtectedData, InitializationMargin>(&(iterator->second.ring));
}

template <typename ProtectedData, std::size_t InitializationMargin>
ConsumerGrantFactory<ProtectedData, InitializationMargin>
Blackboard<ProtectedData, InitializationMargin>::register_consumer(const std::string& label) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto [iterator, inserted] = entries_.try_emplace(label);
  (void)inserted;
  iterator->second.ring.register_consumer();

  return ConsumerGrantFactory<ProtectedData, InitializationMargin>(&(iterator->second.ring));
}

template <typename ProtectedData, std::size_t InitializationMargin>
template <typename Initializer>
ConsumerGrantFactory<ProtectedData, InitializationMargin>
Blackboard<ProtectedData, InitializationMargin>::register_consumer(
    const std::string& label,
    const Initializer& initializer) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto [iterator, inserted] = entries_.try_emplace(label);
  (void)inserted;
  iterator->second.ring.register_consumer(initializer);

  return ConsumerGrantFactory<ProtectedData, InitializationMargin>(&(iterator->second.ring));
}

template <typename ProtectedData, std::size_t InitializationMargin>
bool Blackboard<ProtectedData, InitializationMargin>::all_have_producer() const {
  std::lock_guard<std::mutex> lock(mutex_);

  bool all_registered = true;
  for (const auto& [label, entry] : entries_) {
    (void)label;
    if (!entry.ring.has_producer()) {
      all_registered = false;
    }
  }

  return all_registered;
}

template <typename ProtectedData, std::size_t InitializationMargin>
std::vector<std::string>
Blackboard<ProtectedData, InitializationMargin>::labels_without_producer() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> result;
  result.reserve(entries_.size());

  for (const auto& [label, entry] : entries_) {
    if (!entry.ring.has_producer()) {
      result.push_back(label);
    }
  }

  return result;
}

} // namespace bpe::lfbb
