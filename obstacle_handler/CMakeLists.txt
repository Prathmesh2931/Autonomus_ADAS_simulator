cmake_minimum_required(VERSION 3.8)
project(obstacle_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgproc)

add_executable(obstacle_handler_node src/obstacle_handler_node.cpp)

target_include_directories(obstacle_handler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(obstacle_handler_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

target_link_libraries(obstacle_handler_node ${OpenCV_LIBS})

install(TARGETS obstacle_handler_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()