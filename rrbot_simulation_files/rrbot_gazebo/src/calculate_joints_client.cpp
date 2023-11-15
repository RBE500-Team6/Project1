#include "rclcpp/rclcpp.hpp"
#include "rrbot_gazebo/srv/calculate_joints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Display a helpful message if the number of inputs is incorrect
  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "usage: calculate_joints '{rrbot_pose: {position: {x: X, y: Y, z: Z}}}'");
      return 1;
  }

  // Create the calculate_joints node client
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("calculate_joints_client");
  rclcpp::Client<rrbot_gazebo::srv::CalculateJoints>::SharedPtr client =
    node->create_client<rrbot_gazebo::srv::CalculateJoints>("calculate_joints");

  // Get the Pose position components of the request
  auto request = std::make_shared<rrbot_gazebo::srv::CalculateJoints::Request>();
  request->rrbot_pose.position.x = atoll(argv[1]);
  request->rrbot_pose.position.y = atoll(argv[2]);
  request->rrbot_pose.position.z = atoll(argv[3]);

  // Wait for service, exit if interrupted, wait if not available and print a helpful message
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Send request
  auto result = client->async_send_request(request);
  // Wait for the result.
  // Print the joint values to the screen or display a failure message
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joints: %f %f %f", result.get()->q0,
                result.get()->q1, result.get()->q2);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service calculate_joints");
  }

  rclcpp::shutdown();
  return 0;
}
