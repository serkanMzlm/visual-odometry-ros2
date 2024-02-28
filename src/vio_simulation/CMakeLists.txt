cmake_minimum_required(VERSION 3.8)
project(vio_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY 
    calibration_data
    config
    images
    launch 
    worlds 
  DESTINATION  
    share/${PROJECT_NAME}/
)

ament_package()
