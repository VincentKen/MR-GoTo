#include <memory>
#include "mr_goto/mr_goto_node.hpp"
#include "rclcpp/rclcpp.hpp"

GoToNode::GoToNode(rclcpp::NodeOptions options) : Node("goto", options) {
    // Create instance of GoTo class
    goto_ = std::make_shared<mr::GoTo>();
    
    this->declare_parameter<std::string>("map", "line");
    this->declare_parameter<std::string>("pos_estim", "ekf");
    this->get_parameter("map", map_param_);
    this->get_parameter("pos_estim", pos_estim_param_);

    // Init goal set state
    goal_set = false;

    // sub_ground_truth_ = create_subscription<nav_msgs::msg::Odometry>(
    //     "ground_truth",
    //     10, std::bind(&GoToNode::callback_ground_truth, this, std::placeholders::_1));
    // RCLCPP_INFO(this->get_logger(), "subscribed to ground_truth");

    if(pos_estim_param_ == "ekf"){
        sub_pose_ = create_subscription<nav_msgs::msg::Odometry>(
            "pose_estimate",
            10, std::bind(&GoToNode::callback_pose, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "subscribed to pose_estimate");
    }else{
        sub_pose_ = create_subscription<nav_msgs::msg::Odometry>(
            "ground_truth",
            10, std::bind(&GoToNode::callback_pose, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "subscribed to ground_truth");
    }

    sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose",
        10, std::bind(&GoToNode::callback_goal_pose, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscribed to goal_pose");

    sub_laser_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "base_scan",
        10, std::bind(&GoToNode::callback_laser, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "subscribed to base_scan");

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); //publisher for velocity command

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(40000),
        std::bind(&GoToNode::timer_callback, this)
    );

    //timer for map publisher
    map_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&GoToNode::map_timer_callback, this)
    );

    //publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("nav_msgs/Path", 10);

    // Load the map from the PNG file here
    std::string install_loc = std::getenv("MR_DIR");
    if(map_param_ == "line"){
        map_loc_ = install_loc + "/ws02/src/mr_goto/config/world/bitmaps/line.png";
    }else{
        map_loc_ = install_loc + "/ws02/src/mr_goto/config/world/bitmaps/cave.png";
    }
    map_ = cv::imread(map_loc_, cv::IMREAD_GRAYSCALE);
    if (map_.empty()) {
        std::string message = "Failed to load map image from " + map_loc_;
        RCLCPP_ERROR(this->get_logger(), message.c_str());
        // Handle error...
    } else {
        
        
    }
}

void GoToNode::timer_callback() {
    
}

void GoToNode::map_timer_callback() {
    auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    // Fill out the header
    msg->header.stamp = this->now();
    msg->header.frame_id = "map";
    // Set the map resolution (meters per pixel)
    msg->info.resolution = 0.03;
    // Set the map origin
    msg->info.origin.position.x = -map_.cols / 2.0 * msg->info.resolution;
    msg->info.origin.position.y = -map_.rows / 2.0 * msg->info.resolution;
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;
    // Set the map size
    msg->info.width = map_.cols;
    msg->info.height = map_.rows;
    // Convert the map image data to an OccupancyGrid
    msg->data.resize(msg->info.width * msg->info.height);
    for (int y = 0; y < map_.rows; ++y) {
        for (int x = 0; x < map_.cols; ++x) {
            // Flip Y
            int flipped_y = map_.rows - y - 1;
            // Copy pixel data
            msg->data[flipped_y * msg->info.width + x] = 100 - (map_.at<unsigned char>(y, x) / 255.0 * 100);
        }
    }

    map_pub_->publish(*msg);

    if (!figure_) {
        figure_ = new tuw::Figure(std::string("GoTo"));
        double min_x = - (map_.cols * msg->info.resolution / 2);
        double min_y = - (map_.rows * msg->info.resolution / 2);
        double max_x = -min_x;
        double max_y = -min_y;
        int height = max_x - min_x;
        int width = max_y - min_y;
        int width_pixels = map_.cols;
        int height_pixels = map_.rows;

        figure_->init(width_pixels, height_pixels, min_y, max_y, min_x, max_x, M_PI, 1, 1, map_loc_);
        // cv::namedWindow(figure_->title(), 1);
        // cv::moveWindow(figure_->title(), 20, 20);
        
    }
    // cv::imshow(figure_->title(), figure_->view());
    // cv::waitKey(1);
}

void GoToNode::callback_pose(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double yaw;
    tuw::QuaternionToYaw(msg->pose.pose.orientation, yaw);
    pose_ = tuw::Pose2D(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    if(goal_set){
        //auto publish_msg = goto_->goto_goal_straight(pose_, pose_goal_);
        auto publish_msg = goto_->goto_goal_avoid(pose_, pose_goal_, scan_);
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
    
    if (figure_) {
        tuw::Poses2D waypoints = goto_->pathfinder_waypoints(pose_, pose_goal_, figure_);

        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "nav_msgs/Path";

        for (auto p : waypoints) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.frame_id = "geometry_msgs/PoseStamped"; // no idea what a frame is or what this frame_id is supposed to be
            pose_msg.header.stamp = this->now();
            geometry_msgs::msg::Pose pose;
            p.copyToROSPose<geometry_msgs::msg::Pose>(pose);
            pose_msg.pose = pose;
            path.poses.push_back(pose_msg);
        }
        path_pub_->publish(path);
    }

    RCLCPP_INFO(this->get_logger(), ("Goal position: x=" + std::to_string(pose_goal_.get_x()) + " y=" + std::to_string(pose_goal_.get_y())).c_str());
}


void GoToNode::callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    scan_ = msg;

    size_t n_measurements = scan_->ranges.size();
    laser_measurments_.resize(n_measurements);
    double x, y, angle;
    
    for ( size_t i = 0; i < n_measurements; i++ ) {
      angle = scan_->angle_min + scan_->angle_increment * i;
      x = 0.15 + cos(angle) * scan_->ranges[i];
      y = sin(angle) * scan_->ranges[i];
      laser_measurments_[i] = cv::Vec<double, 3> (x, y, 1.0);
    }
    //RCLCPP_INFO(this->get_logger(), ("Min laser scan: " + std::to_string(std::min_element(scan_->ranges.begin(), scan_->ranges.end())[0])).c_str());

}