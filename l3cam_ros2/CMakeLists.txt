cmake_minimum_required(VERSION 3.5)
project(l3cam_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(l3cam_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include/l3cam_ros2
  /usr/local/lib/libl3cam
  /usr/local/include/libl3cam
)
FIND_LIBRARY(l3cam_LIBRARIES libL3Cam.a /usr/local/lib/libl3cam/)

add_executable(l3cam_ros2_node src/l3cam_ros2_node.cpp)
ament_target_dependencies(l3cam_ros2_node "rclcpp" "l3cam_interfaces")
target_link_libraries(l3cam_ros2_node ${l3cam_LIBRARIES})

add_executable(network_configuration src/network_configuration.cpp)
ament_target_dependencies(network_configuration "rclcpp" "l3cam_interfaces")
target_link_libraries(network_configuration ${l3cam_LIBRARIES})

add_executable(lidar_configuration src/lidar_configuration.cpp)
ament_target_dependencies(lidar_configuration "rclcpp" "l3cam_interfaces")
target_link_libraries(lidar_configuration ${l3cam_LIBRARIES})

add_executable(polarimetric_configuration src/polarimetric_configuration.cpp)
ament_target_dependencies(polarimetric_configuration "rclcpp" "l3cam_interfaces")
target_link_libraries(polarimetric_configuration ${l3cam_LIBRARIES})

add_executable(rgb_configuration src/rgb_configuration.cpp)
ament_target_dependencies(rgb_configuration "rclcpp" "l3cam_interfaces")
target_link_libraries(rgb_configuration ${l3cam_LIBRARIES})

add_executable(thermal_configuration src/thermal_configuration.cpp)
ament_target_dependencies(thermal_configuration "rclcpp" "l3cam_interfaces")
target_link_libraries(thermal_configuration ${l3cam_LIBRARIES})

add_executable(allied_wide_configuration src/allied_wide_configuration.cpp)
ament_target_dependencies(allied_wide_configuration "rclcpp" "l3cam_interfaces")
target_link_libraries(allied_wide_configuration ${l3cam_LIBRARIES})

add_executable(allied_narrow_configuration src/allied_narrow_configuration.cpp)
ament_target_dependencies(allied_narrow_configuration "rclcpp" "l3cam_interfaces")
target_link_libraries(allied_narrow_configuration ${l3cam_LIBRARIES})

add_executable(lidar_stream src/lidar_stream.cpp src/sensor_stream.cpp)
ament_target_dependencies(lidar_stream "rclcpp" "l3cam_interfaces" "sensor_msgs" "cv_bridge")
target_link_libraries(lidar_stream ${l3cam_LIBRARIES})

add_executable(polarimetric_wide_stream src/polarimetric_wide_stream.cpp src/sensor_stream.cpp)
ament_target_dependencies(polarimetric_wide_stream "rclcpp" "l3cam_interfaces" "sensor_msgs" "cv_bridge")
target_link_libraries(polarimetric_wide_stream ${l3cam_LIBRARIES})

add_executable(rgb_narrow_stream src/rgb_narrow_stream.cpp src/sensor_stream.cpp)
ament_target_dependencies(rgb_narrow_stream "rclcpp" "l3cam_interfaces" "sensor_msgs" "cv_bridge")
target_link_libraries(rgb_narrow_stream ${l3cam_LIBRARIES})

add_executable(thermal_stream src/thermal_stream.cpp src/sensor_stream.cpp)
ament_target_dependencies(thermal_stream "rclcpp" "l3cam_interfaces" "sensor_msgs" "cv_bridge")
target_link_libraries(thermal_stream ${l3cam_LIBRARIES})

install(TARGETS 
    l3cam_ros2_node
    network_configuration
    lidar_configuration
    polarimetric_configuration
    rgb_configuration
    thermal_configuration
    allied_wide_configuration
    allied_narrow_configuration
    lidar_stream
    polarimetric_wide_stream
    rgb_narrow_stream
    thermal_stream
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()