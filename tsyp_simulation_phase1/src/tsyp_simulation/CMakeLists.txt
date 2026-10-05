cmake_minimum_required(VERSION 3.8)
project(tsyp_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NetworkPacket.msg"
  "msg/NetworkHop.msg"
  "msg/BeaconState.msg"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY tsyp_simulation/
  DESTINATION ${PYTHON_INSTALL_DIR}/tsyp_simulation
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY launch worlds config resource models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tsyp_simulation/network_emulator_node.py
  tsyp_simulation/writer_robot_node.py
  tsyp_simulation/beacon_manager_node.py
  tsyp_simulation/gateway_node.py
  tsyp_simulation/command_post_dashboard_node.py
  tsyp_simulation/executor_robot_node.py
  tsyp_simulation/scenario_orchestrator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
