#include <cstddef>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>

#include "block_plugin_engine/iblock_plugin.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "bpe_system_test/twist_checker_inspection.hpp"

namespace bpe_system_test::test {
namespace {

class TwistPipelineTest : public ::testing::Test {
protected:
  TwistPipelineTest()
    : loader_("block_plugin_engine", "bpe::IBlockPlugin") {}

  std::shared_ptr<bpe::IBlockPlugin> load(const std::string& type) {
    return loader_.createSharedInstance(type);
  }

  pluginlib::ClassLoader<bpe::IBlockPlugin> loader_;
};

TEST_F(TwistPipelineTest, TypedPluginsExchangeRosMessageThroughBlackboard) {
  auto producer = load("bpe_system_test/TwistProducerBlock");
  auto transform = load("bpe_system_test/TwistTransformBlock");
  auto checker = load("bpe_system_test/TwistCheckerBlock");

  ASSERT_NE(producer, nullptr);
  ASSERT_NE(transform, nullptr);
  ASSERT_NE(checker, nullptr);

  auto* producer_timer = dynamic_cast<bpe::ITimerBlock*>(producer.get());
  auto* transform_timer = dynamic_cast<bpe::ITimerBlock*>(transform.get());
  auto* checker_timer = dynamic_cast<bpe::ITimerBlock*>(checker.get());
  auto* inspection = dynamic_cast<ITwistCheckerInspection*>(checker.get());

  ASSERT_NE(producer_timer, nullptr);
  ASSERT_NE(transform_timer, nullptr);
  ASSERT_NE(checker_timer, nullptr);
  ASSERT_NE(inspection, nullptr);

  const auto producer_config = YAML::Load(R"(
blackboard_output: typed_pipeline_raw_twist
)");

  const auto transform_config = YAML::Load(R"(
blackboard_input: typed_pipeline_raw_twist
blackboard_output: typed_pipeline_processed_twist
linear_x_scale: 2.0
linear_y_scale: -1.0
angular_z_offset: 0.25
)");

  const auto checker_config = YAML::Load(R"(
blackboard_input: typed_pipeline_processed_twist
)");

  ASSERT_EQ(producer->configure(producer_config), bpe::BlockResult::SUCCESS);
  ASSERT_EQ(transform->configure(transform_config), bpe::BlockResult::SUCCESS);
  ASSERT_EQ(checker->configure(checker_config), bpe::BlockResult::SUCCESS);

  ASSERT_EQ(producer->start(), bpe::BlockResult::SUCCESS);
  ASSERT_EQ(transform->start(), bpe::BlockResult::SUCCESS);
  ASSERT_EQ(checker->start(), bpe::BlockResult::SUCCESS);

  constexpr std::size_t kSampleCount = 128;

  for (std::size_t sample = 0; sample < kSampleCount; ++sample) {
    ASSERT_EQ(producer_timer->execute(), bpe::ExecutionResult::SUCCESS);
    ASSERT_EQ(transform_timer->execute(), bpe::ExecutionResult::SUCCESS);
    ASSERT_EQ(checker_timer->execute(), bpe::ExecutionResult::SUCCESS);
  }

  EXPECT_EQ(inspection->checked_samples(), kSampleCount);
  EXPECT_EQ(inspection->errors(), 0U);
  EXPECT_EQ(inspection->missed_samples(), 0U);

  EXPECT_EQ(checker->stop(), bpe::BlockResult::SUCCESS);
  EXPECT_EQ(transform->stop(), bpe::BlockResult::SUCCESS);
  EXPECT_EQ(producer->stop(), bpe::BlockResult::SUCCESS);

  EXPECT_EQ(checker->cleanup(), bpe::BlockResult::SUCCESS);
  EXPECT_EQ(transform->cleanup(), bpe::BlockResult::SUCCESS);
  EXPECT_EQ(producer->cleanup(), bpe::BlockResult::SUCCESS);
}

} // namespace
} // namespace bpe_system_test::test
