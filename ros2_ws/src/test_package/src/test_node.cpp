#include <cstdio>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
  TestNode()
  : Node("test_node")
  {
    // Log a message every second to prove the node is active
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&TestNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Gitea Workflow Verification Node Started");
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Workflow is running...");
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();
  
  // Spin for 3 seconds then exit cleanly to simulate a successful run
  // inside a test script, or keep running if launched manually.
  rclcpp::spin_some(node);
  
  rclcpp::shutdown();
  return 0;
}