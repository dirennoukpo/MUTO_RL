cmake_minimum_required(VERSION 3.8)
project(muto_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(muto_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)

add_executable(depth_processor_node src/depth_processor_node.cpp)
ament_target_dependencies(depth_processor_node rclcpp std_msgs sensor_msgs muto_msgs geometry_msgs nav_msgs pcl_ros pcl_conversions)

add_executable(lidar_processor_node src/lidar_processor_node.cpp)
ament_target_dependencies(lidar_processor_node rclcpp std_msgs sensor_msgs muto_msgs geometry_msgs nav_msgs pcl_ros pcl_conversions)

install(TARGETS
  depth_processor_node
  lidar_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
