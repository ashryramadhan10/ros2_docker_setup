#include "ros2_tutorial/timer_callback_node.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace Tutorial {
    
    TimerCallbackNode::TimerCallbackNode(const std::string& node_name) : Node{node_name}, counter_{0} {
        this->timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TimerCallbackNode::print_hello, this));
    }

    void TimerCallbackNode::print_hello() {
        RCLCPP_INFO(this->get_logger(), "Hello %d", this->counter_);
        this->counter_++;
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tutorial::TimerCallbackNode>("timer_callback_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}