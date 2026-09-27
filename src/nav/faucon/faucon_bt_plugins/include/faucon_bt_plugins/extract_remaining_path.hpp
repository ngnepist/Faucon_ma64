#ifndef FAUCON_BT_PLUGINS__EXTRACT_REMAINING_PATH_HPP_
#define FAUCON_BT_PLUGINS__EXTRACT_REMAINING_PATH_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace faucon_bt_plugins
{

class ExtractRemainingPath : public BT::SyncActionNode
{
public:
  ExtractRemainingPath(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace faucon_bt_plugins

#endif