#include <memory>
#include "mr_goto/mr_goto_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
