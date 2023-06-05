#ifndef MR_GOTO_NODE_
#define MR_GOTO_NODE_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tuw_geometry/tuw_geometry.hpp>

class GoToNode : public rclcpp::Node
{
public:
    __attribute__((visibility("default"))) GoToNode(rclcpp::NodeOptions options);

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_; /// subscriber ground truth pose (simulation)
    nav_msgs::msg::Odometry::SharedPtr ground_truth_;                           /// local copy of the last ground truth pose

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    tuw::Pose2D pose_goal_;
    
    void timer_callback();
    void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif