#ifndef MR_GOTO_NODE_
#define MR_GOTO_NODE_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "nav2_msgs/srv/load_map.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tuw_geometry/tuw_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
    const std::string window_name_ = "GoTo Window";

    std::string map_param_;
    std::string pos_estim_param_;

    cv::Matx33d Mw2m_;             /// transformation world to map
    cv::Matx33d Mm2w_;             /// transformation map to world

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    // Ground-truth robot pose
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_; /// subscriber ground truth pose (simulation)
    tuw::Pose2D pose_;                                                  /// local copy of the last ground truth pose

    // Goal pose
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    tuw::Pose2D pose_goal_;
    bool goal_set;  // stores if a goal was already set by the user
    bool pose_set;

    // Command velocity
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;          /// velocity command pulisher

    // Laser scanner
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;                   /// laser subscriber
    sensor_msgs::msg::LaserScan::SharedPtr scan_;  /// local copy of the last scan
    std::vector<cv::Vec < double, 3 > > laser_measurments_;  /// laser measurments in cartesian robot coordinates

    
    void timer_callback();
    void map_timer_callback();
    void callback_pose(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    std::shared_ptr<mr::GoTo> goto_;         /// pointer to the actual particle filter

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; //map publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    cv::Mat map_; //map matrix
    tuw::Figure* figure_;
    std::string map_loc_;
};

#endif