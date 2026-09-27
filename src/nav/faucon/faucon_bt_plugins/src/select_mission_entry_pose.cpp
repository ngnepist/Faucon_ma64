#include "faucon_bt_plugins/select_mission_entry_pose.hpp"

#include <cmath>
#include "behaviortree_cpp/bt_factory.h"

namespace faucon_bt_plugins
{

SelectMissionEntryPose::SelectMissionEntryPose(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList SelectMissionEntryPose::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("mission_path"),
    BT::InputPort<unsigned int>("closest_index"),
    BT::InputPort<double>(
      "end_restart_distance",
      1.0,
      "Restart mission from beginning when remaining path distance is below this threshold"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("entry_pose"),
    BT::OutputPort<unsigned int>("entry_index")
  };
}

BT::NodeStatus SelectMissionEntryPose::tick()
{
  nav_msgs::msg::Path mission_path;
  unsigned int closest_index;
  double end_restart_distance;

  if (!getInput("mission_path", mission_path)) {
    RCLCPP_ERROR(node_->get_logger(), "SelectMissionEntryPose: missing mission_path");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("closest_index", closest_index)) {
    RCLCPP_ERROR(node_->get_logger(), "SelectMissionEntryPose: missing closest_index");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("end_restart_distance", end_restart_distance)) {
    RCLCPP_ERROR(node_->get_logger(), "SelectMissionEntryPose: missing end_restart_distance");
    return BT::NodeStatus::FAILURE;
  }

  if (mission_path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "SelectMissionEntryPose: mission_path is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (closest_index >= mission_path.poses.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SelectMissionEntryPose: closest_index=%u out of range, path size=%zu",
      closest_index,
      mission_path.poses.size());
    return BT::NodeStatus::FAILURE;
  }

  if (end_restart_distance < 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SelectMissionEntryPose: end_restart_distance must be >= 0");
    return BT::NodeStatus::FAILURE;
  }

  double remaining_distance = 0.0;

  for (std::size_t i = closest_index; i + 1 < mission_path.poses.size(); ++i) {
    const auto & p1 = mission_path.poses[i].pose.position;
    const auto & p2 = mission_path.poses[i + 1].pose.position;

    remaining_distance += std::hypot(
      p2.x - p1.x,
      p2.y - p1.y);
  }

  geometry_msgs::msg::PoseStamped entry_pose;
  unsigned int entry_index;

  if (remaining_distance <= end_restart_distance) {
    entry_index = 0;
    entry_pose = mission_path.poses.front();

    RCLCPP_INFO(
      node_->get_logger(),
      "SelectMissionEntryPose: remaining_distance=%.3f m <= %.3f m -> RESTART FROM BEGINNING",
      remaining_distance,
      end_restart_distance);
  } else {
    entry_index = closest_index;
    entry_pose = mission_path.poses[closest_index];

    RCLCPP_INFO(
      node_->get_logger(),
      "SelectMissionEntryPose: remaining_distance=%.3f m > %.3f m -> RESUME AT CLOSEST POSE index=%u",
      remaining_distance,
      end_restart_distance,
      closest_index);
  }

  setOutput("entry_pose", entry_pose);
  setOutput("entry_index", entry_index);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace faucon_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<faucon_bt_plugins::SelectMissionEntryPose>(
    "SelectMissionEntryPose");
}