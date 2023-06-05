#include "mr_goto/mr_goto_node.hpp"

GoToNode::GoToNode(rclcpp::NodeOptions options) : Node("goto", options) {
    sub_ground_truth_ = create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth",
        10, std::bind(&GoToNode::callback_ground_truth, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscribed to ground_truth");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&GoToNode::timer_callback, this)
    );
}

void GoToNode::timer_callback() {
    RCLCPP_INFO(this->get_logger(), "MR GOTO Timer Callback");
}

void GoToNode::callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->ground_truth_ = msg;
    RCLCPP_INFO(this->get_logger(), std::to_string(msg->pose.pose.position.x).data());
}