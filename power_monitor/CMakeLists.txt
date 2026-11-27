cmake_minimum_required(VERSION 3.8)
project(power_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pr2_msgs REQUIRED)
find_package(Boost REQUIRED)

# ROS 1: dynamic_reconfigure (cfg/PowerMonitor.cfg); ROS 2: the parameter power_state_estimator_.
add_executable(power_monitor
   src/power_monitor.cpp
   src/power_state_estimator.cpp
   src/observation.cpp)
target_include_directories(power_monitor PRIVATE include)
target_link_libraries(power_monitor rclcpp::rclcpp ${pr2_msgs_TARGETS} Boost::boost)

install(TARGETS power_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()
