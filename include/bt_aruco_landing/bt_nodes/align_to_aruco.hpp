#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace bt_aruco_landing
{

class AlignToAruco : public BT::SyncActionNode
{
public:
  AlignToAruco(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;


private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  float prev_dx_{0.0}, prev_dy_{0.0}, prev_yaw_error_{0.0};
  float integral_dx_{0.0}, integral_dy_{0.0}, integral_yaw_error_{0.0};
  rclcpp::Time prev_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace bt_aruco_landing
