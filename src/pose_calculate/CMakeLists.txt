cmake_minimum_required(VERSION 3.8)
project(pose_calculate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)


file(GLOB_RECURSE SRC_FILE ${CMAKE_SOURCE_DIR}/*.cpp)

add_executable(pose_calculate_node ${SRC_FILE})

ament_target_dependencies(
  pose_calculate_node
  geometry_msgs
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  Boost
  cv_bridge
)

install(
  TARGETS pose_calculate_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
