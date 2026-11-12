// Copyright 2026 Heliodoro Tejada
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "dwb_core/publisher.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using namespace std::chrono_literals;

class PublisherTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    namespace_ = std::string("/dwb_publisher_test/") + info->name();
    node_ = std::make_shared<nav2_util::LifecycleNode>("controller_server", namespace_);
    listener_ = std::make_shared<rclcpp::Node>("listener", namespace_);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_->get_node_base_interface());
    executor_->add_node(listener_);
  }

  void TearDown() override
  {
    if (publisher_) {
      publisher_->on_deactivate();
      publisher_->on_cleanup();
    }
    executor_.reset();
    marker_sub_.reset();
    evaluation_sub_.reset();
    publisher_.reset();
    listener_.reset();
    node_.reset();
  }

  void configure(bool publish_evaluation = true, bool publish_trajectories = true)
  {
    node_->declare_parameter("FollowPath.publish_evaluation", publish_evaluation);
    node_->declare_parameter("FollowPath.publish_trajectories", publish_trajectories);
    publisher_ = std::make_unique<dwb_core::DWBPublisher>(node_, "FollowPath");
    publisher_->on_configure();
    publisher_->on_activate();
  }

  void subscribe_markers()
  {
    marker_sub_ = listener_->create_subscription<visualization_msgs::msg::MarkerArray>(
      "marker", 1,
      [this](visualization_msgs::msg::MarkerArray::SharedPtr message) {
        markers_ = message;
      });
  }

  void subscribe_evaluation()
  {
    evaluation_sub_ = listener_->create_subscription<dwb_msgs::msg::LocalPlanEvaluation>(
      "evaluation", 1,
      [this](dwb_msgs::msg::LocalPlanEvaluation::SharedPtr message) {
        evaluation_ = message;
      });
  }

  bool spin_until(const std::function<bool()> & condition, std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  }

  bool discovered(size_t markers, size_t evaluations)
  {
    return spin_until(
      [this, markers, evaluations]() {
        return node_->count_subscribers(namespace_ + "/marker") == markers &&
        node_->count_subscribers(namespace_ + "/evaluation") == evaluations;
      });
  }

  std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> make_evaluation()
  {
    auto result = std::make_shared<dwb_msgs::msg::LocalPlanEvaluation>();
    result->header.frame_id = "odom";
    result->header.stamp = node_->now();
    result->twists.resize(3);
    result->best_index = 0;
    result->worst_index = 1;
    result->twists[0].total = 1.0;
    result->twists[1].total = 3.0;
    result->twists[2].total = -1.0;
    for (auto & score : result->twists) {
      score.traj.poses.resize(2);
      score.traj.poses[1].x = 0.25;
    }
    return result;
  }

  std::string namespace_;
  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr listener_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<dwb_core::DWBPublisher> publisher_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<dwb_msgs::msg::LocalPlanEvaluation>::SharedPtr evaluation_sub_;
  visualization_msgs::msg::MarkerArray::SharedPtr markers_;
  dwb_msgs::msg::LocalPlanEvaluation::SharedPtr evaluation_;
};

TEST_F(PublisherTest, MarkersWithoutEvaluationSubscriber)
{
  configure();
  subscribe_markers();
  ASSERT_TRUE(discovered(1, 0));
  publisher_->publishEvaluation(make_evaluation());
  ASSERT_TRUE(spin_until([this]() {return markers_ != nullptr;}));
  ASSERT_EQ(markers_->markers.size(), 3u);
  EXPECT_EQ(markers_->markers[0].ns, "ValidTrajectories");
  EXPECT_EQ(markers_->markers[2].ns, "InvalidTrajectories");
  EXPECT_EQ(markers_->markers[0].header.frame_id, "odom");
  ASSERT_EQ(markers_->markers[0].points.size(), 2u);
  EXPECT_DOUBLE_EQ(markers_->markers[0].points[1].x, 0.25);
}

TEST_F(PublisherTest, MarkersWhenEvaluationPublicationDisabled)
{
  configure(false, true);
  subscribe_markers();
  ASSERT_TRUE(discovered(1, 0));
  publisher_->publishEvaluation(make_evaluation());
  ASSERT_TRUE(spin_until([this]() {return markers_ != nullptr;}));
  EXPECT_EQ(markers_->markers.size(), 3u);
}

TEST_F(PublisherTest, EvaluationWithoutMarkerSubscriber)
{
  configure();
  subscribe_evaluation();
  ASSERT_TRUE(discovered(0, 1));
  publisher_->publishEvaluation(make_evaluation());
  ASSERT_TRUE(spin_until([this]() {return evaluation_ != nullptr;}));
  EXPECT_EQ(evaluation_->twists.size(), 3u);
  EXPECT_EQ(evaluation_->best_index, 0);
  EXPECT_EQ(evaluation_->worst_index, 1);
}

TEST_F(PublisherTest, BothSubscribersReceiveTheirMessages)
{
  configure();
  subscribe_markers();
  subscribe_evaluation();
  ASSERT_TRUE(discovered(1, 1));
  publisher_->publishEvaluation(make_evaluation());
  ASSERT_TRUE(spin_until([this]() {return markers_ && evaluation_;}));
  EXPECT_EQ(markers_->markers.size(), evaluation_->twists.size());
}

TEST_F(PublisherTest, TrajectoryPublicationCanBeDisabled)
{
  configure(true, false);
  subscribe_markers();
  subscribe_evaluation();
  ASSERT_TRUE(discovered(1, 1));
  publisher_->publishEvaluation(make_evaluation());
  ASSERT_TRUE(spin_until([this]() {return evaluation_ != nullptr;}));
  EXPECT_FALSE(spin_until([this]() {return markers_ != nullptr;}, 200ms));
}

TEST_F(PublisherTest, NullEvaluationDoesNotPublish)
{
  configure();
  subscribe_markers();
  subscribe_evaluation();
  ASSERT_TRUE(discovered(1, 1));
  publisher_->publishEvaluation(nullptr);
  EXPECT_FALSE(spin_until([this]() {return markers_ || evaluation_;}, 200ms));
}
