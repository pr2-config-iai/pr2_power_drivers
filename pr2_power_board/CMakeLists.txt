cmake_minimum_required(VERSION 3.8)
project(pr2_power_board)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/PowerBoardCommand.srv
  srv/PowerBoardCommand2.srv
)

# TODO(ros2 port): power_node / power_node2 (src/power_node), the driver that talks to the power
# board over the network, and the scripts are not ported yet. Only the service definitions are
# built, for rqt_pr2_dashboard.

ament_export_dependencies(rosidl_default_runtime)
ament_package()
