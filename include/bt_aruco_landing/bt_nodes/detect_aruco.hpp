#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <optional>

namespace bt_aruco_landing
{

class DetectAruco : public BT::SyncActionNode
{
public:
  DetectAruco(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  std::optional<geometry_msgs::msg::PoseStamped> detected_pose_;
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;  
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace bt_aruco_landing
