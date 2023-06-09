#ifndef MR_GOTO_HPP_
#define MR_GOTO_HPP_

#include <tuw_geometry/tuw_geometry.hpp>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "mr_goto/mr_goto_bfs.hpp"

namespace mr
{
    using namespace std;
    class GoTo
    {
    public:
        GoTo();
        geometry_msgs::msg::Twist goto_goal_straight(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal);
        std::vector<tuw::Point2D> pathfinder_waypoints(tuw::Pose2D pose_robot, tuw::Pose2D pose_goal, tuw::Figure *figure);

    private:
        tuw::Point2D pathfinder_grid_to_figure(const tuw::Point2D *p, const tuw::Figure *figure);
        tuw::Point2D pathfinder_figure_to_grid(const tuw::Point2D *p, const tuw::Figure *figure);
        void pathfinder_calculate_occupancy_grid(tuw::Figure *figure);

        bool background_filled_ = false;
        cv::Mat background_;
        cv::Mat grid_;
    };
}

#endif // MR_GOTO_HPP_
