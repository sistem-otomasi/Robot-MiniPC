cmake_minimum_required(VERSION 3.5)
project(embedded_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mqtt_client_cpp REQUIRED)

# Create the main executable
add_executable(embedded_bridge_node src/embedded_bridge_node.cpp)

# Link dependencies
ament_target_dependencies(embedded_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  mqtt_client_cpp
)

# Install
install(TARGETS embedded_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
