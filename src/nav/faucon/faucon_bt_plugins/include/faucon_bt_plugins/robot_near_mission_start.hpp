#ifndef FAUCON_BT_PLUGINS__ROBOT_NEAR_MISSION_START_HPP_
#define FAUCON_BT_PLUGINS__ROBOT_NEAR_MISSION_START_HPP_

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace faucon_bt_plugins
{
    class RobotNearMissionStart: public nav2_behavior_tree::BtServiceNode<std_srvs::srv::Trigger>
    {
    public:
    RobotNearMissionStart(const std::string & service_node_name, const BT::NodeConfiguration & conf);

    BT::NodeStatus on_completion(std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
    };
}  // namespace faucon_bt_plugins

#endif  // FAUCON_BT_PLUGINS__ROBOT_NEAR_MISSION_START_HPP_