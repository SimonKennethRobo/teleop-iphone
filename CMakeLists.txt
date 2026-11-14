cmake_minimum_required(VERSION 3.8)
project(teleop_iphone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Build the main node
add_executable(osc_node
  src/osc_node.cpp
)

ament_target_dependencies(osc_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
)

target_link_libraries(osc_node
  nlohmann_json::nlohmann_json
  pthread
)

# Build the teleop node
add_executable(teleop_iphone
  src/teleop_iphone.cpp
)

ament_target_dependencies(teleop_iphone
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2
  ocs2_msgs
)

target_link_libraries(teleop_iphone nlohmann_json::nlohmann_json)

# Install targets
install(TARGETS
  osc_node
  teleop_iphone
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/teleop_iphone_zmq.py
  DESTINATION lib/${PROJECT_NAME}
)

# Use the ROS build interpreter even when the shell's python3 is a different
# Python version without rclpy. Keep the source script runnable with Python.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS scripts/robot_state_feedback.py)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/scripts/robot_state_feedback.py" feedback_script)
string(REGEX REPLACE "^#![^\n]*" "#!${Python3_EXECUTABLE}" feedback_script "${feedback_script}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/robot_state_feedback.py" "${feedback_script}")
install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/robot_state_feedback.py"
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(robot_feedback test/test_robot_feedback.py TIMEOUT 45
    ENV TELEOP_TEST_BIN_DIR=$<TARGET_FILE_DIR:teleop_iphone>)
  ament_add_pytest_test(embodilab_udp test/test_embodilab_udp.py TIMEOUT 45
    ENV TELEOP_TEST_BIN_DIR=$<TARGET_FILE_DIR:teleop_iphone>)
  ament_add_pytest_test(teleop_mpc test/test_teleop_mpc.py TIMEOUT 90
    ENV TELEOP_TEST_BIN_DIR=$<TARGET_FILE_DIR:teleop_iphone>)
endif()

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
