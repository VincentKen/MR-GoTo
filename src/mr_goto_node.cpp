#include "mr_goto/mr_goto_node.hpp"


GoToNode::GoToNode(rclcpp::NodeOptions options) : Node("goto", options) {
    // Create instance of GoTo class
    goto_ = std::make_shared<mr::GoTo>();

    goal_set = false;
    
    sub_ground_truth_ = create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth",
        10, std::bind(&GoToNode::callback_ground_truth, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscribed to ground_truth");

    sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",
        10, std::bind(&GoToNode::callback_goal_pose, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscribed to goal_pose");

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); //publisher for velocity command

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
    double yaw;
    tuw::QuaternionToYaw(msg->pose.pose.orientation, yaw);
    ground_truth_ = tuw::Pose2D(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    //RCLCPP_INFO(this->get_logger(), std::to_string(msg->pose.pose.position.x).data());

    if(goal_set){
        auto publish_msg = goto_->goto_goal_straight(ground_truth_, pose_goal_);
        //RCLCPP_INFO(this->get_logger(), std::to_string(publish_msg.angular.z).data());
        cmd_vel_pub_->publish(publish_msg);
    }
}

void GoToNode::callback_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_set = true;
    double yaw;
    tuw::QuaternionToYaw(msg->pose.orientation, yaw);
    pose_goal_ = tuw::Pose2D(msg->pose.position.x, msg->pose.position.y, yaw);
    
    RCLCPP_INFO(this->get_logger(), ("Goal position: x=" + std::to_string(pose_goal_.get_x()) + " y=" + std::to_string(pose_goal_.get_y())).c_str());
}
