#ifndef MR_GOTO_NODE_
#define MR_GOTO_NODE_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class GoToNode : public rclcpp::Node
{
public:
    __attribute__((visibility("default"))) GoToNode(rclcpp::NodeOptions options);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_; /// subscriber ground truth pose (simulation)
    nav_msgs::msg::Odometry::SharedPtr ground_truth_;                           /// local copy of the last ground truth pose
    
    void timer_callback();
    void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif