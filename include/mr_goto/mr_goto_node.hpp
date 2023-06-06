#ifndef MR_GOTO_NODE_
#define MR_GOTO_NODE_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_msgs/srv/load_map.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tuw_geometry/tuw_geometry.hpp>
#include "mr_goto/mr_goto.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"

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
    rclcpp::TimerBase::SharedPtr map_timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_; /// subscriber ground truth pose (simulation)
    tuw::Pose2D ground_truth_;                                                  /// local copy of the last ground truth pose

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    tuw::Pose2D pose_goal_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;          /// velocity command pulisher

    
    void timer_callback();
    void map_timer_callback();
    void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    std::shared_ptr<mr::GoTo> goto_;         /// pointer to the actual particle filter

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; //map publisher
    cv::Mat map_; //map matrix
};

#endif