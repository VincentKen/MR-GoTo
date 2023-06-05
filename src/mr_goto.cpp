#include "mr_goto/mr_goto.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

GoTo::GoTo()
{
}


geometry_msgs::msg::Twist GoTo::goto_goal_linear(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal)
{
    double orientation_error = angle_difference(pose_goal.theta(), pose_robot.theta());
    auto twist_msg = geometry_msgs::msg::Twist();

    if(orientation_error > 0.1){
        twist_msg.angular.z = orientation_error;
    }
    else{
        twist_msg.angular.z = 0.0;
    }

    twist_msg.linear.x = 0.0;
    
    
    return twist_msg;

}