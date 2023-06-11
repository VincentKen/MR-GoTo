#ifndef MR_GOTO_HPP_
#define MR_GOTO_HPP_

#include <tuw_geometry/tuw_geometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace mr
{
    using namespace std;
    class GoTo
    {
    public:
        GoTo();
        geometry_msgs::msg::Twist goto_goal_straight(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal);
        geometry_msgs::msg::Twist goto_goal_avoid(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal, sensor_msgs::msg::LaserScan::SharedPtr scan);
        float getRangeAtAngle(sensor_msgs::msg::LaserScan::SharedPtr scan, float angle);
    private:
    };
}

#endif // MR_GOTO_HPP_
