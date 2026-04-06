cmake_minimum_required(VERSION 3.8)
project(muto_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(muto_msgs REQUIRED)

include_directories(include)

add_executable(obs_builder_node src/obs_builder_node.cpp)
ament_target_dependencies(obs_builder_node rclcpp std_msgs sensor_msgs geometry_msgs muto_msgs)

add_executable(safety_filter_node src/safety_filter_node.cpp)
ament_target_dependencies(safety_filter_node rclcpp std_msgs sensor_msgs muto_msgs)

install(TARGETS
  obs_builder_node
  safety_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
