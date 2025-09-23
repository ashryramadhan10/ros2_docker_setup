#pragma once

#include "rclcpp/rclcpp.hpp"

namespace Tutorial {

    class TimerCallbackNode : public rclcpp::Node {
        private:
            int counter_;
            rclcpp::TimerBase::SharedPtr timer_;

        public:
            TimerCallbackNode(const std::string& node_name);
            void print_hello();
    };
}