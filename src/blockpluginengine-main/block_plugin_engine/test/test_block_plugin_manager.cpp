#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/logger.hpp>

#include "block_plugin_engine/execution_config.hpp"
#include "block_plugin_engine/iblock_scheduler.hpp"
#include "block_plugin_engine/itimer_block.hpp"
#include "detail/block_plugin_manager.hpp"
#include "test_event_log.hpp"

namespace bpe::test {
namespace {

class TestScheduler : public IBlockScheduler {
public:
  struct Entry {
    std::string name;
    ITimerBlock* block;
    ExecutionConfig config;
  };

  SchedulerResult configure(const SchedulerConfig&) override {
    return SchedulerResult::SUCCESS;
  }

  SchedulerResult add(const std::string& instance_name,
                      ITimerBlock& block,
                      const ExecutionConfig& config) override {
    entries_.push_back({instance_name, &block, config});
    return SchedulerResult::SUCCESS;
  }

  SchedulerResult remove(const std::string& instance_name) override {
    entries_.erase(std::remove_if(entries_.begin(),
                                  entries_.end(),
                                  [&instance_name](const Entry& entry) {
                                    return entry.name == instance_name;
                                  }),
                   entries_.end());

    return SchedulerResult::SUCCESS;
  }

  SchedulerResult start() override {
    running_ = true;
    return SchedulerResult::SUCCESS;
  }

  SchedulerResult stop() override {
    running_ = false;
    return SchedulerResult::SUCCESS;
  }

  SchedulerResult clear() override {
    entries_.clear();
    return SchedulerResult::SUCCESS;
  }

  const std::vector<Entry>& entries() const {
    return entries_;
  }

  ExecutionResult execute(const std::string& instance_name) {
    for (const auto& entry : entries_) {
      if (entry.name == instance_name) {
        return entry.block->execute();
      }
    }

    return ExecutionResult::FAILURE;
  }

private:
  std::vector<Entry> entries_;
  bool running_{false};
};

class BlockPluginManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    clear_events();

    scheduler_ = std::make_shared<TestScheduler>();
    ros_topics_.reset();

    manager_ = std::make_unique<BlockPluginManager>(scheduler_,
                                                    ros_topics_,
                                                    rclcpp::get_logger("bpe_test"),
                                                    std::vector<std::string>{BPE_TEST_PLUGIN_XML});
  }

  std::shared_ptr<TestScheduler> scheduler_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr ros_topics_;
  std::unique_ptr<BlockPluginManager> manager_;
};

YAML::Node load_blocks_config(const std::string& filename) {
  const YAML::Node root = YAML::LoadFile(std::string{BPE_TEST_CONFIG_DIR} + "/" + filename);
  return root["blocks"];
}

TEST_F(BlockPluginManagerTest, LoadsAndRunsValidConfiguration) {
  ASSERT_TRUE(manager_->configure(load_blocks_config("valid.yaml")));

  ASSERT_EQ(scheduler_->entries().size(), 1U);
  EXPECT_EQ(scheduler_->entries()[0].name, "timer");
  EXPECT_EQ(scheduler_->entries()[0].config.policy, ExecutionPolicy::TIMER);
  EXPECT_EQ(scheduler_->entries()[0].config.period, std::chrono::milliseconds{10});

  EXPECT_EQ(events(),
            (std::vector<std::string>{
                "basic_a.configure:10",
                "basic_b.configure:20",
                "timer.configure:periodic",
                "idle_timer.configure:not_scheduled",
            }));

  ASSERT_TRUE(manager_->start());

  EXPECT_EQ(scheduler_->execute("timer"), ExecutionResult::SUCCESS);

  ASSERT_TRUE(manager_->stop());
  ASSERT_TRUE(manager_->cleanup());

  EXPECT_EQ(events(),
            (std::vector<std::string>{
                "basic_a.configure:10",
                "basic_b.configure:20",
                "timer.configure:periodic",
                "idle_timer.configure:not_scheduled",
                "basic_a.start",
                "basic_b.start",
                "timer.start",
                "idle_timer.start",
                "timer.execute",
                "idle_timer.stop",
                "timer.stop",
                "basic_b.stop",
                "basic_a.stop",
                "idle_timer.cleanup",
                "timer.cleanup",
                "basic_b.cleanup",
                "basic_a.cleanup",
            }));
}

TEST_F(BlockPluginManagerTest, RejectsDuplicateNamesBeforeLoadingPlugins) {
  EXPECT_FALSE(manager_->configure(load_blocks_config("duplicate_name.yaml")));
  EXPECT_TRUE(events().empty());
}

TEST_F(BlockPluginManagerTest, RejectsExecutionForNonTimerBlock) {
  EXPECT_FALSE(manager_->configure(load_blocks_config("invalid_execution.yaml")));

  EXPECT_EQ(events(),
            (std::vector<std::string>{
                "basic.configure:42",
                "basic.cleanup",
            }));

  EXPECT_TRUE(scheduler_->entries().empty());
}

TEST_F(BlockPluginManagerTest, RollsBackPreviousBlocksOnConfigureFailure) {
  EXPECT_FALSE(manager_->configure(load_blocks_config("configure_failure.yaml")));

  EXPECT_EQ(events(),
            (std::vector<std::string>{
                "basic.configure:42",
                "failing.configure",
                "basic.cleanup",
            }));
}

TEST_F(BlockPluginManagerTest, StopsPreviouslyStartedBlocksOnStartFailure) {
  ASSERT_TRUE(manager_->configure(load_blocks_config("start_failure.yaml")));

  clear_events();

  EXPECT_FALSE(manager_->start());

  EXPECT_EQ(events(),
            (std::vector<std::string>{
                "basic.start",
                "failing.start",
                "basic.stop",
            }));

  ASSERT_TRUE(manager_->cleanup());
}

} // namespace
} // namespace bpe::test