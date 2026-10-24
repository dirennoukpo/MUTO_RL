cmake_minimum_required(VERSION 3.8)
project(muto_policy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# ONNX Runtime (installed by hand, default /usr/local)
set(ONNXRUNTIME_ROOT "/usr/local" CACHE PATH "ONNX Runtime install root")
find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h
  PATHS ${ONNXRUNTIME_ROOT}/include ${ONNXRUNTIME_ROOT}/include/onnxruntime /usr/include REQUIRED)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  PATHS ${ONNXRUNTIME_ROOT}/lib /usr/lib /usr/lib/aarch64-linux-gnu REQUIRED)

add_executable(muto_policy_node src/muto_policy_node.cpp)
add_executable(muto_policy_selftest src/selftest.cpp)
foreach(target muto_policy_node muto_policy_selftest)
  target_include_directories(${target} PRIVATE include ${ONNXRUNTIME_INCLUDE_DIR})
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_link_libraries(${target} ${ONNXRUNTIME_LIBRARY} nlohmann_json::nlohmann_json)
endforeach()
ament_target_dependencies(muto_policy_node rclcpp sensor_msgs std_msgs geometry_msgs ament_index_cpp)

install(TARGETS muto_policy_node muto_policy_selftest DESTINATION lib/${PROJECT_NAME})

ament_package()
