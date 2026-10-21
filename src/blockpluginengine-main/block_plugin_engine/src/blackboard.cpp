#include "block_plugin_engine/blackboard.hpp"

#include <memory>
#include <utility>

#include <rclcpp/logging.hpp>

#include "detail/runtime_state.hpp"

namespace bpe {
namespace {

std::unique_ptr<BpeBlackboard>& blackboard_instance() {
  static std::unique_ptr<BpeBlackboard> instance = std::make_unique<BpeBlackboard>();
  return instance;
}

} // namespace

BpeBlackboard& the_blackboard() {
  return *blackboard_instance();
}

namespace detail {

void reset_blackboard() {
  auto replacement = std::make_unique<BpeBlackboard>();
  auto& instance = blackboard_instance();

  RCLCPP_DEBUG(rclcpp::get_logger("block_plugin_engine"),
              "Runtime state: destroying blackboard instance");
  instance = std::move(replacement);
  RCLCPP_DEBUG(rclcpp::get_logger("block_plugin_engine"),
              "Runtime state: blackboard instance destroyed and recreated");
}

} // namespace detail
} // namespace bpe
