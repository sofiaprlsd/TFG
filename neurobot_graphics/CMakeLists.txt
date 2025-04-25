cmake_minimum_required(VERSION 3.8)
project(neurobot_graphics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(set_constraints_srv REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(main_node src/main_node.cpp)
ament_target_dependencies(main_node rclcpp std_msgs set_constraints_srv)

add_executable(main_node_sliders src/main_node_sliders.cpp)
ament_target_dependencies(main_node_sliders rclcpp std_msgs set_constraints_srv)

install(TARGETS main_node main_node_sliders DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME} 
PATTERN "*.py" PERMISSIONS OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE)

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