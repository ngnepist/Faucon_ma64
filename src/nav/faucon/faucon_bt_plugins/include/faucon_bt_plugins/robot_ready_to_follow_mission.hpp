#ifndef FAUCON_BT_PLUGINS__ROBOT_READY_TO_FOLLOW_MISSION_HPP_
#define FAUCON_BT_PLUGINS__ROBOT_READY_TO_FOLLOW_MISSION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace faucon_bt_plugins
{

class RobotReadyToFollowMission : public BT::ConditionNode
{
public:
  RobotReadyToFollowMission(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace faucon_bt_plugins

#endif  // FAUCON_BT_PLUGINS__ROBOT_READY_TO_FOLLOW_MISSION_HPP_