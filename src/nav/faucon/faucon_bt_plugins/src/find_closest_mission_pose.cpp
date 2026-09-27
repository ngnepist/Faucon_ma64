#include "faucon_bt_plugins/find_closest_mission_pose.hpp"

#include <limits>

namespace faucon_bt_plugins
{

FindClosestMissionPose::FindClosestMissionPose(const std::string & name, 
    const BT::NodeConfig & config): BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::PortsList FindClosestMissionPose::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("mission_path"),
    BT::InputPort<std::string>("global_frame", std::string("map"), "Global reference frame"),
    BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("closest_pose"),
    BT::OutputPort<unsigned int>("closest_index")
  };
}

BT::NodeStatus FindClosestMissionPose::tick()
{
  nav_msgs::msg::Path mission_path;

  if (!getInput("mission_path", mission_path)) {
    RCLCPP_ERROR(node_->get_logger(), "FindClosestMissionPose: unable to get mission_path.");
    return BT::NodeStatus::FAILURE;
  }

  if (mission_path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "FindClosestMissionPose: mission_path is empty.");
    return BT::NodeStatus::FAILURE;
  }

  std::string global_frame;
  std::string robot_base_frame;

  if (!getInput("global_frame", global_frame) ||
      !getInput("robot_base_frame", robot_base_frame)) {
    RCLCPP_ERROR(node_->get_logger(), "FindClosestMissionPose: unable to get frame names.");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform(
      global_frame,
      robot_base_frame,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "FindClosestMissionPose: TF error: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = transform.transform.translation.x;
  const double robot_y = transform.transform.translation.y;

  double min_distance_sq = std::numeric_limits<double>::max();
  unsigned int closest_index = 0;

  for (unsigned int i = 0; i < mission_path.poses.size(); ++i) {
    const double dx = mission_path.poses[i].pose.position.x - robot_x;
    const double dy = mission_path.poses[i].pose.position.y - robot_y;
    const double distance_sq = dx * dx + dy * dy;

    if (distance_sq < min_distance_sq) {
      min_distance_sq = distance_sq;
      closest_index = i;
    }
  }

  setOutput("closest_pose", mission_path.poses[closest_index]);
  setOutput("closest_index", closest_index);

  RCLCPP_INFO(
    node_->get_logger(),
    "Closest mission pose: index=%u, x=%.3f, y=%.3f",
    closest_index,
    mission_path.poses[closest_index].pose.position.x,
    mission_path.poses[closest_index].pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace faucon_bt_plugins

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<faucon_bt_plugins::FindClosestMissionPose>(
    "FindClosestMissionPose");
}