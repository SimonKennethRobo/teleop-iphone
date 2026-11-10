cmake_minimum_required(VERSION 3.8)
project(teleop_iphone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(ocs2_msgs REQUIRED)

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
  pthread
)

# Build the teleop node
add_executable(teleop_iphone
  src/teleop_iphone.cpp
)

ament_target_dependencies(teleop_iphone
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2
  ocs2_msgs
)

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
  ament_add_pytest_test(teleop_mpc test/test_teleop_mpc.py TIMEOUT 90
    APPEND_ENV TELEOP_TEST_BIN_DIR=$<TARGET_FILE_DIR:teleop_iphone>)
endif()

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
