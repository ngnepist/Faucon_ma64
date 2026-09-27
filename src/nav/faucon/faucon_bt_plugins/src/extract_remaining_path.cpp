#include "faucon_bt_plugins/extract_remaining_path.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace faucon_bt_plugins
{

ExtractRemainingPath::ExtractRemainingPath(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList ExtractRemainingPath::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("mission_path"),
    BT::InputPort<unsigned int>("entry_index"),
    BT::OutputPort<nav_msgs::msg::Path>("remaining_path")
  };
}

BT::NodeStatus ExtractRemainingPath::tick()
{
  nav_msgs::msg::Path mission_path;
  unsigned int entry_index;

  if (!getInput("mission_path", mission_path)) {
    RCLCPP_ERROR(node_->get_logger(), "ExtractRemainingPath: missing mission_path");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("entry_index", entry_index)) {
    RCLCPP_ERROR(node_->get_logger(), "ExtractRemainingPath: missing entry_index");
    return BT::NodeStatus::FAILURE;
  }

  if (mission_path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "ExtractRemainingPath: mission_path is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (entry_index >= mission_path.poses.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ExtractRemainingPath: entry_index=%u out of range, path size=%zu",
      entry_index,
      mission_path.poses.size());
    return BT::NodeStatus::FAILURE;
  }

  nav_msgs::msg::Path remaining_path;
  remaining_path.header = mission_path.header;

  remaining_path.poses.insert(
    remaining_path.poses.end(),
    mission_path.poses.begin() + entry_index,
    mission_path.poses.end());

  setOutput("remaining_path", remaining_path);

  RCLCPP_INFO(
    node_->get_logger(),
    "ExtractRemainingPath: entry_index=%u, %zu/%zu poses remaining",
    entry_index,
    remaining_path.poses.size(),
    mission_path.poses.size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace faucon_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<faucon_bt_plugins::ExtractRemainingPath>(
    "ExtractRemainingPath");
}