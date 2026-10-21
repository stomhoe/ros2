#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "block_plugin_engine/block_config.hpp"
#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "block_plugin_engine/typed_blackboard.hpp"
#include "bpe_system_test/twist_checker_inspection.hpp"

namespace bpe_system_test {
namespace {

bool message_is_expected(const geometry_msgs::msg::TwistStamped& message) noexcept {
  const double value = -message.twist.angular.x;

  return message.header.frame_id == "bpe_system_test" &&
         message.twist.linear.x == 2.0 * value &&
         message.twist.linear.y == -2.0 * value &&
         message.twist.linear.z == 3.0 * value &&
         message.twist.angular.x == -value &&
         message.twist.angular.y == -2.0 * value &&
         message.twist.angular.z == (0.5 * value) + 0.25;
}

} // namespace

class TwistCheckerBlock final
  : public bpe::IBlockPlugin,
    public bpe::ITimerBlock,
    public ITwistCheckerInspection {
public:
  using Message = geometry_msgs::msg::TwistStamped;

  bpe::BlockResult configure(const bpe::BlockConfig& config) override {
    bpe::BlockResult result = bpe::BlockResult::SUCCESS;

    try {
      if (!config["blackboard_input"]) {
        result = bpe::BlockResult::FAILURE;
      } else {
        blackboard_input_ = config["blackboard_input"].as<std::string>();
        consumer_.bind(blackboard_input_);
      }
    } catch (...) {
      result = bpe::BlockResult::FAILURE;
    }

    return result;
  }

  bpe::ExecutionResult execute() override {
    bpe::ExecutionResult result = bpe::ExecutionResult::SUCCESS;

    auto input = consumer_.read_access();

    if (input.has_new_value()) {
      bool valid = message_is_expected(input.value());

      if (input.has_missed()) {
        missed_samples_.fetch_add(
          static_cast<std::size_t>(input.missed()),
          std::memory_order_relaxed);
      }

      if (last_sequence_ == 0) {
        if (input.sequence() != 1) {
          valid = false;
        }
      } else if (input.sequence() != last_sequence_ + 1) {
        valid = false;
      }

      last_sequence_ = input.sequence();

      if (!valid) {
        errors_.fetch_add(1, std::memory_order_relaxed);
      }

      checked_samples_.fetch_add(1, std::memory_order_release);
    }

    return result;
  }

  std::size_t checked_samples() const noexcept override {
    return checked_samples_.load(std::memory_order_acquire);
  }

  std::size_t errors() const noexcept override {
    return errors_.load(std::memory_order_acquire);
  }

  std::size_t missed_samples() const noexcept override {
    return missed_samples_.load(std::memory_order_acquire);
  }

private:
  bpe::TypedConsumer<Message> consumer_;
  std::string blackboard_input_;
  std::uint64_t last_sequence_{0};
  std::atomic<std::size_t> checked_samples_{0};
  std::atomic<std::size_t> errors_{0};
  std::atomic<std::size_t> missed_samples_{0};
};

} // namespace bpe_system_test

PLUGINLIB_EXPORT_CLASS(
  bpe_system_test::TwistCheckerBlock,
  bpe::IBlockPlugin)
