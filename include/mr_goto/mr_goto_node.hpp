#ifndef MR_GOTO_NODE_
#define MR_GOTO_NODE_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tuw_geometry/tuw_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "mr_goto/mr_goto.hpp"

namespace mr
{
  class GoTo;
}
class GoToNode : public rclcpp::Node
{
public:
    __attribute__((visibility("default"))) GoToNode(rclcpp::NodeOptions options);

private:
    rclcpp::TimerBase::SharedPtr timer_;

    // Ground-truth robot pose
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_; /// subscriber ground truth pose (simulation)
    tuw::Pose2D ground_truth_;                                                  /// local copy of the last ground truth pose

    // Goal pose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    tuw::Pose2D pose_goal_;
    bool goal_set;  // stores if a goal was already set by the user

    // Command velocity
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;          /// velocity command pulisher

    // Laser scanner
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;                   /// laser subscriber
    sensor_msgs::msg::LaserScan::SharedPtr scan_;  /// local copy of the last scan
    std::vector<cv::Vec < double, 3 > > laser_measurments_;  /// laser measurments in cartesian robot coordinates

    
    void timer_callback();
    void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    std::shared_ptr<mr::GoTo> goto_;         /// pointer to the actual particle filter
};

#endif