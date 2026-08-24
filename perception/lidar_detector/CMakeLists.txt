cmake_minimum_required(VERSION 3.8)
project(lidar_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

add_executable(lidar_detector src/lidar_detector.cpp)
target_include_directories(lidar_detector PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(lidar_detector ${PCL_LIBRARIES})
ament_target_dependencies(lidar_detector
  rclcpp sensor_msgs visualization_msgs vision_msgs geometry_msgs pcl_conversions)

install(TARGETS lidar_detector DESTINATION lib/${PROJECT_NAME})

ament_package()
