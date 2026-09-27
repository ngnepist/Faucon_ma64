#ifndef FAUCON_BT_PLUGINS__FIND_CLOSEST_MISSION_POSE_HPP_
#define FAUCON_BT_PLUGINS__FIND_CLOSEST_MISSION_POSE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace faucon_bt_plugins
{

class FindClosestMissionPose : public BT::SyncActionNode
{
public:
  FindClosestMissionPose(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace faucon_bt_plugins

#endif // FAUCON_BT_PLUGINS__FIND_CLOSEST_MISSION_POSE_HPP_