#include "bt_aruco_landing/bt_nodes/align_to_aruco.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <cmath>

namespace bt_aruco_landing
{

AlignToAruco::AlignToAruco(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)  // FIXED HERE
{
  node_ = rclcpp::Node::make_shared("align_to_aruco_bt_node");
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/simple_drone/msdk_cmd_vel", rclcpp::QoS(10));
}

BT::PortsList AlignToAruco::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("aruco_pose")
  };
}

BT::NodeStatus AlignToAruco::tick()
{
  rclcpp::spin_some(node_);

  geometry_msgs::msg::PoseStamped marker_pose;
  if (!getInput("aruco_pose", marker_pose)) {
    RCLCPP_WARN(node_->get_logger(), "No ArUco pose input");
    return BT::NodeStatus::FAILURE;
  }

  float dx = marker_pose.pose.position.x;  // Assuming the marker is aligned with the y-axis
  float dy = marker_pose.pose.position.y; // Assuming the marker is aligned with the x-axis

  const float tolerance = 0.1;
  const float kp = 0.1;

  geometry_msgs::msg::Twist cmd_vel;

  bool aligned = true;

  if (std::abs(dx) > tolerance) {
    cmd_vel.linear.y = -kp * dx;
    aligned = false;
  }

  if (std::abs(dy) > tolerance) {
    cmd_vel.linear.x = -kp * dy;
    aligned = false;
  }

  if (aligned) {
    RCLCPP_INFO(node_->get_logger(), "Alignment complete");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Not aligned yet: dx=%.2f, dy=%.2f", dx, dy);
    cmd_vel_pub_->publish(cmd_vel);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace bt_aruco_landing
