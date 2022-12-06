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
#include "beginner_tutorials/srv/input_str.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;
using namespace std::placeholders;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class Pub : public rclcpp::Node {
 public:
  Pub(char *transformation[]) : Node("Publisher"), count_(0) {
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
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
    this->make_transforms(transformation);
    timer_ = this->create_wall_timer(
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
  void make_transforms(char * transformation[]) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(atof(transformation[5]), atof(transformation[6]),
             atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  std::string new_str = "Hey Terps";
  // int freq;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::InputStr>::SharedPtr service;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pub>(argv));
  // RCLCPP_FATAL_STREAM(node->get_logger(), "ROS Shutting Down");
  rclcpp::shutdown();
  return 0;
}
