cmake_minimum_required(VERSION 3.5)
project(rrbot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalculateJoints.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

ament_export_dependencies(rosidl_default_runtime)

add_executable(publisher src/position_publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(switch src/controller_switch.cpp)
ament_target_dependencies(switch rclcpp controller_manager_msgs)

add_executable(server src/calculate_joints_server.cpp)
ament_target_dependencies(server rclcpp)
target_link_libraries(server "${cpp_typesupport_target}")

add_executable(client src/calculate_joints_client.cpp)
ament_target_dependencies(client rclcpp)
target_link_libraries(client "${cpp_typesupport_target}")


install(TARGETS
  publisher
  switch
  server
  client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
