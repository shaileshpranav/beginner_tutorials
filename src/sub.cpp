/**
 * @file sub.cpp
 * @author Shailesh Pranav Rajendran (spraj@umd.edu)
 * @brief A subscriber script
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright MIT License (c) 2022 Shailesh Pranav Rajendran
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class Sub : public rclcpp::Node {
 public:
  Sub() : Node("Subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&Sub::topic_callback, this, _1));
  }

 private:
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "From publisher: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sub>());
  rclcpp::shutdown();
  return 0;
}
