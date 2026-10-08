cmake_minimum_required(VERSION 3.16)
project(franka_cartesian_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# qpOASES is installed locally rather than through the obsolete qpoases_vendor
# package. Override with -DQPOASES_ROOT=/path/to/install if needed.
set(QPOASES_ROOT "" CACHE PATH "qpOASES installation prefix")
if(NOT QPOASES_ROOT)
  if(DEFINED ENV{QPOASES_ROOT} AND NOT "$ENV{QPOASES_ROOT}" STREQUAL "")
    set(QPOASES_ROOT "$ENV{QPOASES_ROOT}")
  elseif(DEFINED ENV{HOME})
    set(QPOASES_ROOT "$ENV{HOME}/alexis_ws/qpoases_install")
  endif()
endif()

find_path(QPOASES_INCLUDE_DIR
  NAMES qpOASES.hpp
  HINTS
    "${QPOASES_ROOT}/include"
    "${QPOASES_ROOT}/include/qpOASES"
  PATH_SUFFIXES qpOASES
)

find_library(QPOASES_LIBRARY
  NAMES qpOASES qpoases
  HINTS
    "${QPOASES_ROOT}/lib"
    "${QPOASES_ROOT}/lib64"
)

if(NOT QPOASES_INCLUDE_DIR OR NOT QPOASES_LIBRARY)
  message(FATAL_ERROR
    "qpOASES was not found. Expected it under '${QPOASES_ROOT}'. "
    "Set -DQPOASES_ROOT=/your/qpOASES/install prefix if installed elsewhere.")
endif()

message(STATUS "qpOASES include: ${QPOASES_INCLUDE_DIR}")
message(STATUS "qpOASES library: ${QPOASES_LIBRARY}")

# Use a version-specific STATIC library so all four controller executables
# embed the matching Kinematics implementation. This prevents an old shared
# libcartesian_control_common.so in a sourced ROS 2 overlay from overriding
# the resolver and causing ABI mismatches/segmentation faults.
add_library(cartesian_control_common_v7 STATIC
  src/common.cpp
  src/kinematics.cpp
)
target_include_directories(cartesian_control_common_v7 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cartesian_control_common_v7 Eigen3::Eigen)
ament_target_dependencies(cartesian_control_common_v7
  rclcpp sensor_msgs geometry_msgs std_msgs kdl_parser orocos_kdl
)

add_executable(ik_velocity_controller src/ik_velocity_node.cpp)
target_link_libraries(ik_velocity_controller cartesian_control_common_v7 Eigen3::Eigen)
ament_target_dependencies(ik_velocity_controller rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(qp_velocity_controller src/qp_velocity_node.cpp)
target_include_directories(qp_velocity_controller PRIVATE ${QPOASES_INCLUDE_DIR})
target_link_libraries(qp_velocity_controller
  cartesian_control_common_v7
  Eigen3::Eigen
  ${QPOASES_LIBRARY}
)
ament_target_dependencies(qp_velocity_controller
  rclcpp sensor_msgs geometry_msgs std_msgs
)

add_executable(pid_position_controller src/pid_position_node.cpp)
target_link_libraries(pid_position_controller cartesian_control_common_v7 Eigen3::Eigen)
ament_target_dependencies(pid_position_controller rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(lmpc_position_controller src/lmpc_position_node.cpp)
target_include_directories(lmpc_position_controller PRIVATE ${QPOASES_INCLUDE_DIR})
target_link_libraries(lmpc_position_controller
  cartesian_control_common_v7
  Eigen3::Eigen
  ${QPOASES_LIBRARY}
)
ament_target_dependencies(lmpc_position_controller
  rclcpp sensor_msgs geometry_msgs std_msgs visualization_msgs
)

# Ensure the runtime linker can find a non-system shared qpOASES install.
get_filename_component(QPOASES_LIBRARY_DIR "${QPOASES_LIBRARY}" DIRECTORY)
set_target_properties(qp_velocity_controller lmpc_position_controller PROPERTIES
  BUILD_RPATH "${QPOASES_LIBRARY_DIR}"
  INSTALL_RPATH "${QPOASES_LIBRARY_DIR}"
)

install(TARGETS
  cartesian_control_common_v7
  ik_velocity_controller
  qp_velocity_controller
  pid_position_controller
  lmpc_position_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
