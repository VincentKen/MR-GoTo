#include "mr_goto/mr_goto.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace mr;
using namespace tuw;
using namespace std;

GoTo::GoTo()
{
}


geometry_msgs::msg::Twist GoTo::goto_goal_straight(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal)
{
    auto twist_msg = geometry_msgs::msg::Twist();

    // Calculate orientation to face to goal
    double orientation_to_goal = Point2D(pose_goal.position() - pose_robot.position()).angle();
    double orientation_error = angle_difference(orientation_to_goal, pose_robot.theta());

    // Calculate distance to goal
    double dist_to_goal = pose_robot.position().distanceTo(pose_goal.position());
    

    // First turn to face to goal 
    // If we are very close to goal orientation_error is not numerically stable -> exclude this case
    if(abs(orientation_error) > 0.01 && dist_to_goal > 0.01){
        twist_msg.angular.z = 2 * orientation_error;
        twist_msg.linear.x = 0.0;
    }
    // Facing goal
    else{
        // If not at goal position drive forward
        if(dist_to_goal > 0.01){
            twist_msg.linear.x = min(0.8, 0.1 + dist_to_goal);
            twist_msg.angular.z = 0.0;
        }
        // At goal position. Rotate to correct goal orientation
        else{
            double final_orientation_error = angle_difference(pose_goal.theta(), pose_robot.theta());
            twist_msg.angular.z = 2 * final_orientation_error;
            twist_msg.linear.x = 0.0;
        }
    }
    return twist_msg;
}