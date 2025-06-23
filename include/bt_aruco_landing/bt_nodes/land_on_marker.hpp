#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <std_msgs/msg/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
namespace bt_aruco_landing
{

class LandOnMarker : public BT::SyncActionNode
{
public:
  LandOnMarker(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gt_pose_sub_;
};

}  // namespace bt_aruco_landing