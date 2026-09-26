cmake_minimum_required(VERSION 3.8)
project(tram_odometry)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tram_vehicle_msgs REQUIRED)

add_executable(odometry_node src/odometry_node.cpp src/estimator.cpp)
target_compile_features(odometry_node PUBLIC cxx_std_17)
target_include_directories(odometry_node PRIVATE include)
ament_target_dependencies(odometry_node
  rclcpp nav_msgs sensor_msgs diagnostic_msgs std_msgs builtin_interfaces
  ament_index_cpp tram_vehicle_msgs)

install(TARGETS odometry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config assets DESTINATION share/${PROJECT_NAME})

ament_package()
