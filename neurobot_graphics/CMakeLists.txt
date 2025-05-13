cmake_minimum_required(VERSION 3.8)
project(neurobot_graphics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(set_constraints_srv REQUIRED)
find_package(hebi_cpp_api REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface QUIET)
find_package(moveit_msgs QUIET)
find_package(moveit_visual_tools QUIET)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(motorControl src/motorControl.cpp)
target_include_directories(motorControl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(main_node src/main_node.cpp)
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(main_node
  rclcpp
  std_msgs
  set_constraints_srv
  rclcpp_action
  ament_index_cpp
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  hebi_cpp_api
  control_msgs
)

ament_target_dependencies(control_node
  rclcpp
  std_msgs
  set_constraints_srv
  rclcpp_action
  ament_index_cpp
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  hebi_cpp_api
  control_msgs
)

target_link_libraries(control_node motorControl)

install(TARGETS main_node control_node main_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
