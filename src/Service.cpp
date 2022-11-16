/**
 * @file Service.cpp
 * @author Shailesh Pranav Rajendran (spraj@umd.edu)
 * @brief A service client script
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright MIT License (c) 2022 Shailesh Pranav Rajendran
 *
 */
#include <chrono>
#include <cstdlib>
#include <memory>

#include "beginner_tutorials/srv/input_str.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc == 1) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "usage: To change string of publisher");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("service_client");
  rclcpp::Client<beginner_tutorials::srv::InputStr>::SharedPtr client =
      node->create_client<beginner_tutorials::srv::InputStr>("service_node");

  auto request = std::make_shared<beginner_tutorials::srv::InputStr::Request>();
  request->inp_msg = argv[1];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "String: %s",
                result.get()->out_msg.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service InputStr");
  }

  rclcpp::shutdown();
  return 0;
}
