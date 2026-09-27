#include "faucon_bt_plugins/robot_near_mission_start.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace faucon_bt_plugins
{
    RobotNearMissionStart::RobotNearMissionStart(const std::string & service_node_name,
    const BT::NodeConfiguration & conf)
    : nav2_behavior_tree::BtServiceNode<std_srvs::srv::Trigger>(service_node_name, conf)
    {
    }

    BT::NodeStatus RobotNearMissionStart::on_completion(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "RobotNearMissionStart: %s", response->message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_WARN(node_->get_logger(), "RobotNearMissionStart: %s", response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }

}  // namespace faucon_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<faucon_bt_plugins::RobotNearMissionStart>(
    "RobotNearMissionStart");
}