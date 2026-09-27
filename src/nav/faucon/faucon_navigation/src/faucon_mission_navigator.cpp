#include "faucon_navigation/faucon_mission_navigator.hpp"
#include <cmath>
#include <limits>
#include "tf2/exceptions.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace faucon_navigation
{
    bool FauconMissionNavigator::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node, 
        std::shared_ptr<nav2_util::OdomSmoother> /* odom_smoother */)
    {
        auto node = parent_node.lock();
        if (!node) {
            return false;
        }
        const std::string prefix = getName() + ".";
        if (!node->has_parameter(prefix + "entry_distance_threshold")) {
            node->declare_parameter(prefix + "entry_distance_threshold", 0.5);
        }
        if (!node->has_parameter(prefix + "entry_yaw_threshold")) {
            node->declare_parameter(prefix + "entry_yaw_threshold", 0.523599);
        }
        if (!node->has_parameter(prefix + "mission_path_blackboard_id")) {
            node->declare_parameter(prefix + "mission_path_blackboard_id", "mission_path");
        }
        node->get_parameter(prefix + "entry_distance_threshold", entry_reached_threshold_);
        node->get_parameter(prefix + "entry_yaw_threshold", entry_yaw_threshold_);
        node->get_parameter(prefix + "mission_path_blackboard_id", mission_path_blackboard_id_);

        tf_buffer_ = bt_action_server_->getBlackboard()->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        bt_action_server_->getBlackboard()->set(entry_distance_threshold_blackboard_id_, entry_reached_threshold_);
        bt_action_server_->getBlackboard()->set(entry_yaw_threshold_blackboard_id_, entry_yaw_threshold_);

        return true;
    }

    std::string FauconMissionNavigator::getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
    {
        auto node = parent_node.lock();
        if (!node) {return "";
        }

        if (!node->has_parameter(getName() + ".default_bt_xml")) {
            node->declare_parameter(getName() + ".default_bt_xml",
            std::string("/Faucon_ma64/install/faucon_config/share/faucon_config/config/nav/faucon_mission.xml"));
        }
        return node->get_parameter(getName() + ".default_bt_xml").as_string();
    }

    bool FauconMissionNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
    {   

        if (goal->mission_path.poses.empty()) {
            RCLCPP_ERROR(logger_, "Mission rejected: mission_path is empty.");
            return false;
        }
        if (goal->mission_path.header.frame_id.empty()) {
            RCLCPP_ERROR(logger_, "Mission rejected: mission_path frame_id is empty.");
            return false;
        }
        mission_path_ = goal->mission_path;
        
        progress_index_ = 0;
        progress_index_initialized_ = false;
        rejoin_backward_ = false;
        rejoin_completed_ = true;
        
        bt_action_server_->getBlackboard()->set(mission_path_blackboard_id_, goal->mission_path);

        RCLCPP_INFO(logger_, "Faucon mission accepted: %zu poses, frame='%s'.",
            goal->mission_path.poses.size(), goal->mission_path.header.frame_id.c_str());
        return true;
    }

    void FauconMissionNavigator::onLoop()
    {
        // if the mission is empty or the tf tree is not available, stop and return
        if (mission_path_.poses.empty() || !tf_buffer_) {
            return;
        }

        // Get the pose of the robot through transformation
        const std::string & global_frame = mission_path_.header.frame_id;
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(global_frame, "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(logger_, "Feedback mission: TF %s -> base_link indisponible: %s",
            global_frame.c_str(), ex.what());
            return;
        }
        // initialisation of index
        unsigned int entry_index = 0;
        unsigned int initial_closest_index = 0;
        try {
            entry_index =bt_action_server_->getBlackboard()->get<unsigned int>("entry_index");
            initial_closest_index = bt_action_server_->getBlackboard()->get<unsigned int>("closest_index");
        } catch (...) {
            return;
        }
        if (entry_index >= mission_path_.poses.size()) {
            return;
        }
        // initialisation of the progress index 
        if (!progress_index_initialized_) {
            progress_index_ = entry_index;
            progress_index_initialized_ = true;
        }
        // Check if the robot try to rejoin the traj or already on the traj 
        if (!rejoin_backward_ && entry_index < initial_closest_index) {
            rejoin_backward_ = true;
            rejoin_completed_ = false;
            RCLCPP_INFO(logger_, "Backward rejoin detected: closest_index=%u, entry_index=%u.",
                initial_closest_index, entry_index);
        }
        // search the position of the robot on the traj
        const double robot_x = transform.transform.translation.x;
        const double robot_y = transform.transform.translation.y;
        const double robot_yaw = tf2::getYaw(transform.transform.rotation);
        const auto & entry_position =mission_path_.poses[entry_index].pose.position;
        const double entry_yaw = tf2::getYaw(mission_path_.poses[entry_index].pose.orientation);
        const double distance_to_entry = std::hypot(entry_position.x - robot_x, entry_position.y - robot_y);
        double yaw_error = entry_yaw - robot_yaw;
        yaw_error = std::atan2( std::sin(yaw_error), std::cos(yaw_error));
        // The robot is on the rejoin phase, so the progression is not take in account
        if (rejoin_backward_ && !rejoin_completed_ && distance_to_entry <= entry_reached_threshold_ && std::abs(yaw_error) <= entry_yaw_threshold_)
        {
            rejoin_completed_ = true;
            RCLCPP_INFO(logger_, "Mission entry reached: index=%u, distance=%.2f m, yaw_error=%.1f deg.",
                entry_index, distance_to_entry, std::abs(yaw_error) * 180.0 / M_PI);
        }
        if (rejoin_backward_ && !rejoin_completed_) {
            double distance_remaining = 0.0;
            for (std::size_t i = entry_index; i + 1 < mission_path_.poses.size(); ++i)
            {
                const auto & p0 = mission_path_.poses[i].pose.position;
                const auto & p1 = mission_path_.poses[i + 1].pose.position;
                distance_remaining += std::hypot( p1.x - p0.x, p1.y - p0.y);
            }

            auto feedback = std::make_shared<ActionT::Feedback>();
            feedback->distance_remaining = static_cast<float>(distance_remaining);

            bt_action_server_->publishFeedback(feedback);
            return;
        }
        std::size_t closest_index = entry_index;
        double min_distance_sq = std::numeric_limits<double>::max();
        // The robot have finished to rejoin the traj so we can compute the progression of the mission
        // Search where the robot is on the traj( the closset index)
        for (std::size_t i = entry_index; i < mission_path_.poses.size(); ++i) {
            const auto & p = mission_path_.poses[i].pose.position;
            const double dx = p.x - robot_x;
            const double dy = p.y - robot_y;
            const double distance_sq = dx * dx + dy * dy;
            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                closest_index = i;
            }
        }
        progress_index_ = std::max(progress_index_, closest_index); // To avoid backward and forward (oscillation) of the progression
        double distance_remaining = 0.0;
        for (std::size_t i = progress_index_; i + 1 < mission_path_.poses.size(); ++i)
        {
            const auto & p0 = mission_path_.poses[i].pose.position;
            const auto & p1 = mission_path_.poses[i + 1].pose.position;
            distance_remaining += std::hypot( p1.x - p0.x, p1.y - p0.y);
        }

        auto feedback = std::make_shared<ActionT::Feedback>();
        feedback->distance_remaining = static_cast<float>(distance_remaining);
        feedback->restarting = rejoin_backward_ && !rejoin_completed_;
        bt_action_server_->publishFeedback(feedback);
    }

    void FauconMissionNavigator::onPreempt(ActionT::Goal::ConstSharedPtr /* goal */)
    {
        RCLCPP_WARN(logger_, "Mission preemption is not supported. Cancel the current mission before sending a new one.");
        bt_action_server_->terminatePendingGoal();
    }

    void FauconMissionNavigator::goalCompleted(ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
    {
        switch (final_bt_status) {
            case nav2_behavior_tree::BtStatus::SUCCEEDED:
            result->success = true;
            result->message = "Faucon mission completed successfully.";
            RCLCPP_INFO(logger_, "%s", result->message.c_str());
            break;

            case nav2_behavior_tree::BtStatus::FAILED:
            result->success = false;
            result->message = "Faucon mission failed.";
            RCLCPP_ERROR(logger_, "%s", result->message.c_str());
            break;

            case nav2_behavior_tree::BtStatus::CANCELED:
            result->success = false;
            result->message = "Faucon mission canceled.";
            RCLCPP_WARN(logger_, "%s", result->message.c_str());
            break;
        }
    }
}  // namespace faucon_navigation

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  faucon_navigation::FauconMissionNavigator,
  nav2_core::NavigatorBase)