cmake_minimum_required(VERSION 3.8)
project(robot_autonomous)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ── Install Python Package ───────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# ── Install script executables ───────────────────────────────────────────────
install(PROGRAMS
  robot_autonomous/gps_driver_node.py
  robot_autonomous/imu_driver_node.py
  robot_autonomous/robot_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install Launch Files ──────────────────────────────────────────────────────
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ── Resource ─────────────────────────────────────────────────────────────────
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# ── Ament Export ─────────────────────────────────────────────────────────────
ament_package()
