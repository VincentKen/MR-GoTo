#ifndef MR_GOTO_NODE_
#define MR_GOTO_NODE_

#include <rclcpp/rclcpp.hpp>

class GoToNode : public rclcpp::Node
{
public:
    __attribute__((visibility("default"))) GoToNode(rclcpp::NodeOptions options);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    
    void timer_callback();
};

#endif