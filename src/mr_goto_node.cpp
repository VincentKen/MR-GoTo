#include "mr_goto/mr_goto_node.hpp"
#include "rclcpp/rclcpp.hpp"

GoToNode::GoToNode(rclcpp::NodeOptions options) : Node("goto", options) {
    // Create instance of GoTo class
    goto_ = std::make_shared<mr::GoTo>();
    
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

    //timer for map publisher
    map_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&GoToNode::map_timer_callback, this)
    );

    //publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    // Load the map from the PNG file here
    map_ = cv::imread("ws02/src/MR-GoTo/config/world/bitmaps/line.png", cv::IMREAD_GRAYSCALE);
    if (map_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map image.");
        // Handle error...
    }
}

void GoToNode::timer_callback() {
    //RCLCPP_INFO(this->get_logger(), "MR GOTO Timer Callback");
}

void GoToNode::map_timer_callback() {
    auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    // Fill out the header
    msg->header.stamp = this->now();
    msg->header.frame_id = "map";  // Adjust as needed
    // Set the map resolution (meters per pixel)
    msg->info.resolution = 0.05;  // Adjust as needed
    // Set the map origin
    msg->info.origin.position.x = 0.0;  // Adjust as needed
    msg->info.origin.position.y = 0.0;  // Adjust as needed
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;
    // Set the map size
    msg->info.width = map_.cols;
    msg->info.height = map_.rows;
    // Convert the map image data to an OccupancyGrid
    msg->data.resize(msg->info.width * msg->info.height);
    for (int y = 0; y < map_.rows; ++y) {
        for (int x = 0; x < map_.cols; ++x) {
            // Here we're assuming that white (255) is free space and black (0) is occupied.
            // Adjust as needed for your map.
            msg->data[y * msg->info.width + x] = 100 - (map_.at<unsigned char>(y, x) / 255.0 * 100);
        }
    }
    // Publish the OccupancyGrid
    map_pub_->publish(*msg);
}

void GoToNode::callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double yaw;
    tuw::QuaternionToYaw(msg->pose.pose.orientation, yaw);
    ground_truth_ = tuw::Pose2D(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    //RCLCPP_INFO(this->get_logger(), std::to_string(msg->pose.pose.position.x).data());

    auto publish_msg = goto_->goto_goal_linear(ground_truth_, pose_goal_);
    RCLCPP_INFO(this->get_logger(), std::to_string(publish_msg.angular.z).data());
    cmd_vel_pub_->publish(publish_msg);
    
}

void GoToNode::callback_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    double yaw;
    tuw::QuaternionToYaw(msg->pose.orientation, yaw);
    pose_goal_ = tuw::Pose2D(msg->pose.position.x, msg->pose.position.y, yaw);
    
    RCLCPP_INFO(this->get_logger(), ("Goal position: x=" + std::to_string(pose_goal_.get_x()) + " y=" + std::to_string(pose_goal_.get_y())).c_str());
}
