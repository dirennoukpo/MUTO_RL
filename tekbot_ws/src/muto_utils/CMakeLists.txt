cmake_minimum_required(VERSION 3.8)
project(muto_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(include)

add_executable(diagnostics_node src/diagnostics_node.cpp)
ament_target_dependencies(diagnostics_node rclcpp std_msgs diagnostic_msgs)

install(TARGETS diagnostics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
