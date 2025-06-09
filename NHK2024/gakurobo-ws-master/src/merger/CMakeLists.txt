cmake_minimum_required(VERSION 3.8)
project(merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

find_package(PCL 1.2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(merger src/merger.cpp)
ament_target_dependencies(merger
  rclcpp
  laser_geometry
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)
target_link_libraries(merger ${PCL_LIBRARIES})

install(TARGETS
  merger
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line_segment skips the linter which checks for copyrights
  # comment the line_segment when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line_segment skips cpplint (only works in a git repo)
  # comment the line_segment when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
