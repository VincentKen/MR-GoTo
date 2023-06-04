#include "mr_goto/mr_goto_node.hpp"

GoToNode::GoToNode(rclcpp::NodeOptions options) : Node("goto", options) {
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&GoToNode::timer_callback, this)
    );
}

void GoToNode::timer_callback() {
    RCLCPP_INFO(this->get_logger(), "MR GOTO Timer Callback");
}