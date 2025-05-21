#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "../src/turtlebot3_drive.hpp"


class Turtlebot3DriveTest : public ::testing::Test
{
protected:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<Turtlebot3Drive>();
  }

  std::shared_ptr<Turtlebot3Drive> node_;
};

TEST_F(Turtlebot3DriveTest, TestNodeInitialization)
{
  // Test basic initialization values
  EXPECT_EQ(node_->get_name(), "turtlebot3_drive_node");
}
