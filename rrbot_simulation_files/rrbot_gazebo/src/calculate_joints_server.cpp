#include "rclcpp/rclcpp.hpp"
#include "rrbot_gazebo/srv/calculate_joints.hpp"                                        

#include <memory>
#include <cmath>


// Called when a request has been received
void calculate_joints(const std::shared_ptr<rrbot_gazebo::srv::CalculateJoints::Request> request,     
                      std::shared_ptr<rrbot_gazebo::srv::CalculateJoints::Response>      response)  
{

  // Constants
  float L1 = 1.5;   // L1
  float L2 = 1;     // L2
  float L3 = 0.5;   // L3
                    // L4

  // Get the Pose position components
  float x = request->rrbot_pose.position.x;
  float y = request->rrbot_pose.position.y;
  float z = request->rrbot_pose.position.z;

  // Calculate q1 first since q0 depends on q1
  float d = (pow(x, 2) + pow(y, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3);
  response->q1 = atan2(sqrt(1 - pow(d, 2)), d);

  // Calculate q0
  float total_angle = atan2(y, x);
  response->q0 = total_angle - response->q1;

  // Calculate q2
  response->q2 = L1 - z;

  // Display a message that a request has been received with the components
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nx: %f" " y: %f" " z: %f",  
              request->rrbot_pose.position.x, request->rrbot_pose.position.y,
              request->rrbot_pose.position.z);
  // Log the response to be sent back
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f %f %f]", response->q0,
              response->q1, response->q2);
}

int main(int argc, char **argv)
{
  // Initialize
  rclcpp::init(argc, argv);

  // Make calculate_joints_server available as a node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("calculate_joints_server");   

  // Create the service and call calculate_joints
  rclcpp::Service<rrbot_gazebo::srv::CalculateJoints>::SharedPtr service =               
    node->create_service<rrbot_gazebo::srv::CalculateJoints>("calculate_joints",  &calculate_joints);   

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate joints.");                     

  rclcpp::spin(node);
  rclcpp::shutdown();
}
