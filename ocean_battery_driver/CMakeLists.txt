cmake_minimum_required(VERSION 3.8)
project(ocean_battery_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pr2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

# The battery server driver and the battery self test (ROS 1: the same two executables).
add_executable(ocean_server src/ocean_battery_driver/server.cpp src/ocean_battery_driver/ocean.cpp)
target_link_libraries(ocean_server
  rclcpp::rclcpp
  ${pr2_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  Boost::program_options)

add_executable(battery_check src/ocean_battery_driver/battery_check.cpp src/ocean_battery_driver/ocean.cpp)
target_link_libraries(battery_check
  rclcpp::rclcpp
  ${pr2_msgs_TARGETS}
  Boost::program_options)

install(TARGETS ocean_server battery_check DESTINATION lib/${PROJECT_NAME})

ament_package()
