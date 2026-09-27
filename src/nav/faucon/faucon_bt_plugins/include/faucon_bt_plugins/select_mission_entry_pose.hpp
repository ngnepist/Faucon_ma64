#ifndef FAUCON_BT_PLUGINS__SELECT_MISSION_ENTRY_POSE_HPP_
#define FAUCON_BT_PLUGINS__SELECT_MISSION_ENTRY_POSE_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace faucon_bt_plugins
{

class SelectMissionEntryPose : public BT::SyncActionNode
{
public:
  SelectMissionEntryPose(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace faucon_bt_plugins

#endif