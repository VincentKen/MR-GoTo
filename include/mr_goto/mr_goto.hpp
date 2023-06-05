#ifndef MR_GOTO_HPP_
#define MR_GOTO_HPP_

#include <tuw_geometry/tuw_geometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

namespace mr
{
    using namespace std;
    class GoTo
    {
    public:
        GoTo();
        geometry_msgs::msg::Twist goto_goal_linear(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal);

    private:
    };
}

#endif // MR_GOTO_HPP_
