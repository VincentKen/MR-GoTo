#ifndef MR_PF_PKG__PARTICLE_FILTER_NODE_HPP_
#define MR_PF_PKG__PARTICLE_FILTER_NODE_HPP_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tuw_geometry/tuw_geometry.hpp>

namespace mr
{
  class ParticleFilter;
}
class ParticleFilterNodeParameter;

/**
 * ROS2 Node which forward incoming and outgoing data to the particle filter
 */
class PFNode : public rclcpp::Node
{
public:
  /// Constructor
  __attribute__((visibility("default"))) PFNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;    /// laser subscriber
  rclcpp::TimerBase::SharedPtr timer_;                                        /// timer to on_time()
  std::shared_ptr<cv::Vec<double, 3>> p_lclick_;                              /// if allocated a left clicked point
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;        /// motion command subscriber
  tuw::Command2DPtr cmd_;                                                     /// local copy of the last command
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_; /// subscriber ground truth pose (simulation)
  nav_msgs::msg::Odometry::SharedPtr ground_truth_;                           /// local copy of the last ground truth pose
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};          /// tf listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                /// tf buffer
  std::string base_frame_;                                                     /// name of the robots base frame id
  std::string map_frame_;                                                      /// name of the map frame id
  unsigned int filter_update_cycle_;                                          /// parameter to define the update cycle time in ms
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_estimate_pub_;   /// pose_estimate publisher
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;  /// initial pose subscriber

  /// Callback function for incoming range measurements
  void callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /// Callback function for incoming twist command measurements
  void callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);

  /// Callback function for incoming Odometry ground truth msgs
  void callback_ground_truth(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// function called by an internal callback x time per second
  void on_timer();

  //calback for init pose
  void callback_init_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  std::shared_ptr<mr::ParticleFilter> filter_;         /// pointer to the actual particle filter
  std::shared_ptr<ParticleFilterNodeParameter> param_; /// pointer to the parameters used by the particle filter

};

#endif // MR_PF_PKG__PARTICLE_FILTER_NODE_HPP_
