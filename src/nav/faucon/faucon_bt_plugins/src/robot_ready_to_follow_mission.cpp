#include "faucon_bt_plugins/robot_ready_to_follow_mission.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

#include "behaviortree_cpp/bt_factory.h"
#include "tf2/exceptions.hpp"
#include "tf2/utils.hpp"

namespace faucon_bt_plugins
{

RobotReadyToFollowMission::RobotReadyToFollowMission(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::PortsList RobotReadyToFollowMission::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
    BT::InputPort<std::string>("global_frame", std::string("map"), "Global reference frame"),
    BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    BT::InputPort<double>("distance_threshold", 0.5, "Maximum distance to mission path in meters"),
    BT::InputPort<double>("yaw_threshold", 0.523599, "Maximum orientation error in radians")
  };
}

BT::NodeStatus RobotReadyToFollowMission::tick()
{
  geometry_msgs::msg::PoseStamped target_pose;
  std::string global_frame;
  std::string robot_base_frame;
  double distance_threshold;
  double yaw_threshold;

  if (!getInput("target_pose", target_pose) ||
      !getInput("global_frame", global_frame) ||
      !getInput("robot_base_frame", robot_base_frame) ||
      !getInput("distance_threshold", distance_threshold) ||
      !getInput("yaw_threshold", yaw_threshold))
  {
    RCLCPP_ERROR(node_->get_logger(), "RobotReadyToFollowMission: unable to read input ports.");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform(
      global_frame,
      robot_base_frame,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "RobotReadyToFollowMission: TF error: %s",
      ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = transform.transform.translation.x;
  const double robot_y = transform.transform.translation.y;

  const double target_x = target_pose.pose.position.x;
  const double target_y = target_pose.pose.position.y;

  const double distance = std::hypot(
    target_x - robot_x,
    target_y - robot_y);

  const double robot_yaw = tf2::getYaw(transform.transform.rotation);
  const double target_yaw = tf2::getYaw(target_pose.pose.orientation);

  const double yaw_error = std::atan2(
    std::sin(target_yaw - robot_yaw),
    std::cos(target_yaw - robot_yaw));

  const bool distance_ok = distance <= distance_threshold;
  const bool yaw_ok = std::abs(yaw_error) <= yaw_threshold;

  RCLCPP_INFO(
    node_->get_logger(),
    "RobotReadyToFollowMission: distance=%.3f m (max=%.3f), yaw_error=%.1f deg (max=%.1f) -> %s",
    distance,
    distance_threshold,
    yaw_error * 180.0 / M_PI,
    yaw_threshold * 180.0 / M_PI,
    (distance_ok && yaw_ok) ? "READY" : "NOT READY");

  return (distance_ok && yaw_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace faucon_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<faucon_bt_plugins::RobotReadyToFollowMission>(
    "RobotReadyToFollowMission");
}