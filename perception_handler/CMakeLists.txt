cmake_minimum_required(VERSION 3.8)
project(perception_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# FIND DEPENDENCIES
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
# =========================
# INCLUDE DIRECTORIES
# =========================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# =========================
# EXECUTABLES
# =========================

add_executable(lane_detection_node
  src/lane_detection.cpp
)

ament_target_dependencies(
  lane_detection_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  lane_detection_node
  ${OpenCV_LIBS}
)

# -----------------------------------

add_executable(lane_detect_pid_control
  src/lane_detect_with_pidcontrol.cpp
)

ament_target_dependencies(
  lane_detect_pid_control
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  lane_detect_pid_control
  ${OpenCV_LIBS}
)

# -----------------------------------

add_executable(sliding_window
  src/sliding_window_lane_detect.cpp
)

ament_target_dependencies(
  sliding_window
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  sliding_window
  ${OpenCV_LIBS}
)

# -----------------------------------

add_executable(hsv_tuner
  src/hsv_tuner.cpp
)

ament_target_dependencies(
  hsv_tuner
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(
  hsv_tuner
  ${OpenCV_LIBS}
)

add_executable(temporal_lane_detect
  src/temporal_sobel_lookahead_detect.cpp
)

ament_target_dependencies(
  temporal_lane_detect
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
)

target_link_libraries(
  temporal_lane_detect
  ${OpenCV_LIBS}
)

add_executable(multi_confidence_lane_detect
  src/multi_factor_confidense_lane_detect.cpp
)

ament_target_dependencies(
  multi_confidence_lane_detect
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
)

target_link_libraries(
  multi_confidence_lane_detect
  ${OpenCV_LIBS}
)
# =========================
# INSTALL TARGETS
# =========================

install(TARGETS
  lane_detection_node
  lane_detect_pid_control
  sliding_window
  hsv_tuner
  temporal_lane_detect
  multi_confidence_lane_detect
  DESTINATION lib/${PROJECT_NAME}
)

# Optional
install(DIRECTORY include
  DESTINATION include
)

# =========================
# TESTING
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()