/**
 * @file pub.cpp
 * @author Shailesh Pranav Rajendran (spraj@umd.edu)
 * @brief A publisher script
 * @version 0.1
 * @date 2022-11-01
 *
 * @copyright MIT License (c) 2022 Shailesh Pranav Rajendran
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "beginner_tutorials/srv/input_str.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Pub : public rclcpp::Node {
 public:
  Pub() : Node("Publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "This parameter is received from launch";
    this->declare_parameter("freq", 3, param_desc);
    auto freq = this->get_parameter("freq").get_parameter_value().get<int>();
    if (freq != 3)
      RCLCPP_ERROR_STREAM(this->get_logger(), "Publish Frequency changed");
    RCLCPP_INFO_STREAM(this->get_logger(), "Frequency value : " << freq);

    try {
      service = this->create_service<beginner_tutorials::srv::InputStr>(
          "service_node",
          std::bind(&Pub::input_string, this, std::placeholders::_1,
                    std::placeholders::_2));
    } catch (...) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error in calling service");
      RCLCPP_FATAL_STREAM(this->get_logger(), "String might not be changed");
    }
    timer_ =
        this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>((1000 / freq))),
          std::bind(&Pub::timer_callback, this));
  }
  void input_string(
      const std::shared_ptr<beginner_tutorials::srv::InputStr::Request> request,
      std::shared_ptr<beginner_tutorials::srv::InputStr::Response> response) {
    response->out_msg = request->inp_msg;
    new_str = response->out_msg;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: %s",
                request->inp_msg.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: %s",
                response->out_msg.c_str());
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = new_str;
    if (new_str != "Hey Terps")
      RCLCPP_WARN_STREAM(this->get_logger(), "Change to Input String");
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Change to String");
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  std::string new_str = "Hey Terps";
  // int freq;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::InputStr>::SharedPtr service;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<Pub>();
  rclcpp::spin(node);
  RCLCPP_FATAL_STREAM(node->get_logger(), "ROS Shutting Down");
  rclcpp::shutdown();
  return 0;
}
