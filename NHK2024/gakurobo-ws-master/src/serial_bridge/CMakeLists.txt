cmake_minimum_required(VERSION 3.8)
project(serial_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(obstacle_msgs REQUIRED)
find_package(ball_camera_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(bridge src/bridge.cpp)
ament_target_dependencies(bridge
  rclcpp
  serial
  obstacle_msgs
  std_msgs
  ball_camera_msgs
)

install(TARGETS
  bridge
  DESTINATION lib/${PROJECT_NAME})

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
