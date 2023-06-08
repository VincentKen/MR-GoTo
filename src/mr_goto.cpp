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

geometry_msgs::msg::Twist GoTo::goto_goal_avoid(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal, sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    static auto twist_msg = geometry_msgs::msg::Twist();

    double max_speed = 0.8;
    double unsafe_dist_val = 0.6;

    // Calculate orientation to face to goal
    double orientation_to_goal = Point2D(pose_goal.position() - pose_robot.position()).angle();
    double orientation_error = angle_difference(orientation_to_goal, pose_robot.theta());

    // Calculate distance to goal
    double dist_to_goal = pose_robot.position().distanceTo(pose_goal.position());

    
    // Check if seomething in the sensors range is too close to keep moving safely
    // Additionally calculate sum of left and right ranges to know where obstacle is
    static bool obstacle_in_front = false;
    size_t nr_of_ranges = scan->ranges.size();
    double check_angle = M_PI/2;
    size_t start_index = (size_t)(nr_of_ranges / 2 - check_angle/2 / scan->angle_increment);
    size_t end_index = (size_t)(nr_of_ranges / 2 + check_angle/2 / scan->angle_increment);
    double left_ranges_sum = 0.0;
    double right_ranges_sum = 0.0;


   bool obstacle_was_in_front = obstacle_in_front;
   obstacle_in_front = false;
    // Right side
    for (size_t i = start_index; i <= nr_of_ranges / 2; i++) {
        // If we are turning because there was an obstacle reset obstacle_in_front only with a hysteresis
        if(obstacle_was_in_front){
            if (scan->ranges[i] < unsafe_dist_val * 1.2) {
            obstacle_in_front = true;
            }
        }
        else {
            if (scan->ranges[i] < unsafe_dist_val) {
            obstacle_in_front = true;
            }
        }
        right_ranges_sum += scan->ranges[i];
    }
    // Left side
    for (size_t i = nr_of_ranges / 2; i <= end_index; i++) {
        // If we are turning because there was an obstacle reset obstacle_in_front only with a hysteresis
        if(obstacle_was_in_front){
            if (scan->ranges[i] < unsafe_dist_val * 1.2) {
            obstacle_in_front = true;
            }
        }
        else {
            if (scan->ranges[i] < unsafe_dist_val) {
            obstacle_in_front = true;
            }
        }
        left_ranges_sum += scan->ranges[i];
    }
    
    // Drive towards the goal 
    // If we are very close to goal orientation_error is not numerically stable -> exclude this case
    // Only if there is no obstacle in front
    if(!obstacle_in_front && abs(orientation_error) > 0.01 && dist_to_goal > 0.01){
        // Check if we are not turning into a wall
        if(orientation_error < 0 ? (right_ranges_sum > 120) : (left_ranges_sum > 120)){
            twist_msg.angular.z = 2 * orientation_error;
        }
        else{
            twist_msg.angular.z = 0.0;
        }
        twist_msg.linear.x = min(2.0, 0.1 + dist_to_goal);
    }
    // Facing goal or obstacle
    else{
        // If not at goal position drive forward or avoid obstacle 
        if(dist_to_goal > 0.01){
            // Obstacle infront -> turn
            if(obstacle_in_front){
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = right_ranges_sum < left_ranges_sum ? 0.5 : -0.5;
            }
            // Drive straight
            else {
                twist_msg.linear.x = min(max_speed, 0.1 + dist_to_goal);
                twist_msg.angular.z = 0.0;
            }
            
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
