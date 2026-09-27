#ifndef FAUCON_NAVIGATION__FAUCON_MISSION_NAVIGATOR_HPP_
#define FAUCON_NAVIGATION__FAUCON_MISSION_NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "nav2_core/behavior_tree_navigator.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "faucon_interfaces/action/navigate_faucon_mission.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

namespace faucon_navigation
{

class FauconMissionNavigator : public nav2_core::BehaviorTreeNavigator<faucon_interfaces::action::NavigateFauconMission>
{
public:
  using ActionT = faucon_interfaces::action::NavigateFauconMission;

  FauconMissionNavigator() = default;
  ~FauconMissionNavigator() override = default;

  std::string getName() override
  {
    return "navigate_faucon_mission";
  }

protected:
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  std::string getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) override;

  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
  void onLoop() override;
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  void goalCompleted(
    ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override;

private:
  std::string mission_path_blackboard_id_;
  nav_msgs::msg::Path mission_path_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  bool rejoin_backward_{false};
  bool rejoin_completed_{true};
  double entry_reached_threshold_{0.5};
  double entry_yaw_threshold_{0.523599};
  std::size_t progress_index_{0};
  bool progress_index_initialized_{false};
  std::string entry_distance_threshold_blackboard_id_{"entry_distance_threshold"};
  std::string entry_yaw_threshold_blackboard_id_{"entry_yaw_threshold"};
};

}  // namespace faucon_navigation

#endif