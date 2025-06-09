cmake_minimum_required(VERSION 3.8)
project(detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

find_package(PCL 1.2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(obstacle_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(clusterer src/clusterer.cpp)
target_link_libraries(clusterer ${PCL_LIBRARIES})

add_library(detector_impl src/detector.cpp)
ament_target_dependencies(detector_impl
  rclcpp
  sensor_msgs
  pcl_conversions
  geometry_msgs
  obstacle_msgs
  std_msgs
)
target_link_libraries(detector_impl clusterer ${PCL_LIBRARIES})

add_library(visualizer_impl src/visualizer.cpp)
ament_target_dependencies(visualizer_impl
  rclcpp
  obstacle_msgs
  visualization_msgs
)

add_library(filter_impl src/filter.cpp)
ament_target_dependencies(filter_impl
  rclcpp
  obstacle_msgs
  std_msgs
)

add_executable(detector src/nodes/detector_node.cpp)
ament_target_dependencies(detector
  rclcpp
)
target_link_libraries(detector
  detector_impl
)

add_executable(visualizer src/nodes/visualizer_node.cpp)
ament_target_dependencies(visualizer
  rclcpp
)
target_link_libraries(visualizer
  visualizer_impl
)

add_executable(filter src/nodes/filter_node.cpp)
ament_target_dependencies(filter
  rclcpp
)
target_link_libraries(filter
  filter_impl
)

install(TARGETS
  detector
  visualizer
  filter
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
