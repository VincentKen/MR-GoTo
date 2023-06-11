#include "mr_goto/mr_goto.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include "mr_goto/mr_goto_bfs.hpp"

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
    static int turning_from_obstacle = 0;

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
        if(orientation_error < 0 ? (getRangeAtAngle(scan, -M_PI/2) > 0.7) : (getRangeAtAngle(scan, M_PI/2) > 0.7)){
            twist_msg.angular.z = 2 * orientation_error;
            //std::cout << "turning to goal" << endl;
        }
        else{
            twist_msg.angular.z = 0.0;
        }
        // Drive when not very close to goal
        if(dist_to_goal > 1.0){
            twist_msg.linear.x = min(max_speed, 0.1 + dist_to_goal);
        } 
        // If very close to goal just turn and not drive. Otherwise robot circles around goal sometimes
        else {
            twist_msg.linear.x= 0.0;
        }
    }
    // Facing goal or obstacle
    else{
        // If not at goal position drive forward or avoid obstacle 
        if(dist_to_goal > 0.01){
            // Obstacle infront -> turn
            if(obstacle_in_front){
                twist_msg.linear.x = 0.0;
                if(turning_from_obstacle == 0){
                    turning_from_obstacle = right_ranges_sum < left_ranges_sum ? 1 : -1;
                }
                twist_msg.angular.z = turning_from_obstacle * 0.5;
                //std::cout << "turning from obstacle" << endl;
            }
            // Drive straight
            else {
                twist_msg.linear.x = min(max_speed, 0.1 + dist_to_goal);
                twist_msg.angular.z = 0.0;
                turning_from_obstacle = 0;

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

float GoTo::getRangeAtAngle(sensor_msgs::msg::LaserScan::SharedPtr scan, float angle)
{
    // Convert angle to index in the ranges array
    float angleIncrement = scan->angle_increment;
    float startAngle = scan->angle_min;
    int index = static_cast<int>((angle - startAngle) / angleIncrement);

    // Return the range value at the calculated index
    if (index >= 0 && static_cast<size_t>(index) < scan->ranges.size())
        return scan->ranges[index];
    else
        return 0.0f; // Return 0 if index is out of range
}

std::vector<tuw::Pose2D> GoTo::pathfinder_waypoints(tuw::Pose2D start, tuw::Pose2D target, tuw::Figure* figure) {
    pathfinder_calculate_occupancy_grid(figure);
    tuw::Point2D s = figure->w2m(start.position());
    tuw::Point2D t = figure->w2m(target.position());
    s.set_x(int(s.x()));
    s.set_y(int(s.y()));
    t.set_x(int(t.x()));
    t.set_y(int(t.y()));

    s = pathfinder_figure_to_grid(&s, figure);
    t = pathfinder_figure_to_grid(&t, figure);
    std::vector<tuw::Pose2D> waypoints = mr::GoToBFS::search(tuw::Pose2D(s, start.theta()), tuw::Pose2D(t, target.theta()), &grid_);
    std::vector<tuw::Pose2D> translated_waypoints;
    
    for (auto p : waypoints) {
        tuw::Point2D np = pathfinder_grid_to_figure(&p.position(), figure);
        np = figure->m2w(np);
        p.set_x(np.x());
        p.set_y(np.y());
        translated_waypoints.emplace_back(p);
    }
    return translated_waypoints;
}

tuw::Point2D GoTo::pathfinder_grid_to_figure(const tuw::Point2D *p, const tuw::Figure *figure) {
    int cols = (figure->max_x() - figure->min_x()); // these are the columns we see on the map visualization
    int rows = (figure->max_y() - figure->min_y()); // these are the rows
    double col_width = figure->width()/cols;
    double row_height = figure->height()/rows;
    int x = p->x()*col_width + col_width/2;
    int y = p->y()*row_height + row_height/2;
    return tuw::Point2D(x, y);
}

tuw::Point2D GoTo::pathfinder_figure_to_grid(const tuw::Point2D *p, const tuw::Figure *figure) {
    int cols = (figure->max_x() - figure->min_x()); // these are the columns we see on the map visualization
    int rows = (figure->max_y() - figure->min_y()); // these are the rows
    double col_width = int(figure->width()/cols);
    double row_height = int(figure->height()/rows);
    int x = int(p->x()/col_width);
    int y = int(p->y()/row_height);
    return tuw::Point2D(x, y);
}


void GoTo::pathfinder_calculate_occupancy_grid(tuw::Figure *figure) {
    int cols = (figure->max_x() - figure->min_x()); // these are the columns we see on the map visualization
    int rows = (figure->max_y() - figure->min_y()); // these are the rows
    // waypoints will be placed in the center of these cells

    int col_width = int(figure->width()/cols);
    int row_width = int(figure->height()/rows);

    cv::resize(figure->background_image(), background_, cv::Size(figure->height(), figure->width()), cv::INTER_AREA);

    cv::Mat oc = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC1);
    cv::Mat cp(cv::Size(background_.cols, background_.rows), background_.type());
    background_.copyTo(cp);

    // quick and dirty convolution
    // if a pixel is not white, make its neighbours also not white, do this a certain amount of times
    // for (int k = 0; k < 5; k++) {
    //     for (int i = 1; i < background_.rows - 1; i++) {
    //         for (int j = 1; j < background_.cols - 1; j++) {
    //             auto v = background_.at<cv::Vec3b>(i, j);
    //             if (v[0] != 255 && v[1] != 255 && v[2] != 255) {
    //                 cp.at<cv::Vec3b>(i - 1, j) = cv::Vec3b(0, 0, 0);
    //                 cp.at<cv::Vec3b>(i + 1, j) = cv::Vec3b(0, 0, 0);
    //                 cp.at<cv::Vec3b>(i, j - 1) = cv::Vec3b(0, 0, 0);
    //                 cp.at<cv::Vec3b>(i, j + 1) = cv::Vec3b(0, 0, 0);
    //             }
    //         }
    //     }
    //     cp.copyTo(background_);
    // }

    for (int i = 0; i < background_.rows; i++) {
        for (int j = 0; j < background_.cols; j++) {
            auto v = background_.at<cv::Vec3b>(i, j);
            if (v[0] != 255 && v[1] != 255 && v[2] != 255) {
                cv::Point p(int(j/col_width), int(i/row_width));
                oc.at<uchar>(p) = 255;
            }
        }
    }

    // std::cout << "Occupancy:" << std::endl;
    // for (int r = 0; r < rows; r++) {
    //     for (int c = 0; c < cols; c++) {
    //         if (oc.at<uchar>(r, c) < 100) {
    //             std::cout << " " << int(oc.at<uchar>(r, c)) << "  ";    
    //         } else {
    //             std::cout << int(oc.at<uchar>(r, c)) << " ";
    //         }
            
    //     }
    //     std::cout << std::endl;
    // }
    grid_ = oc;
}