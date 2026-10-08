cmake_minimum_required(VERSION 3.16)
project(low_level LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)

add_library(low_level_controllers SHARED
  src/joint_velocity_controller.cpp
  src/joint_impedance_controller.cpp
  src/dynamics_observer_controller.cpp
)
target_include_directories(low_level_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(low_level_controllers
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  franka_semantic_components
)

pluginlib_export_plugin_description_file(controller_interface low_level_controllers.xml)


add_executable(collision_behavior_configurator src/collision_behavior_configurator.cpp)
ament_target_dependencies(collision_behavior_configurator
  rclcpp
  franka_msgs
)

add_executable(gripper_control_node src/gripper_control_node.cpp)
target_include_directories(gripper_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gripper_control_node
  rclcpp
  rclcpp_action
  std_msgs
  franka_msgs
)

install(TARGETS
  low_level_controllers
  gripper_control_node
  collision_behavior_configurator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/readiness_gate.py
  scripts/dynamics_identification_experiment.py
  DESTINATION lib/${PROJECT_NAME}
)

# The identification executable imports this pure-Python fitter at runtime.
install(FILES scripts/stribeck_model.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES low_level_controllers.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  franka_semantic_components
)

ament_package()
