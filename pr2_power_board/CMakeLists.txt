cmake_minimum_required(VERSION 3.8)
project(pr2_power_board)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pr2_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/PowerBoardCommand.srv
  srv/PowerBoardCommand2.srv
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# The power board driver (ROS 1's power_node2; power_node, the simulator and the scripts are not
# ported).
add_executable(power_node2 src/power_node/power_node2.cpp)
target_include_directories(power_node2 PRIVATE include)
target_link_libraries(power_node2
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  ${pr2_msgs_TARGETS}
  Boost::program_options)
install(TARGETS power_node2 DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
