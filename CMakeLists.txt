cmake_minimum_required(VERSION 3.8)
project(dual_cam_yolo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn videoio highgui)
find_package(vision_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  /opt/ros/jazzy/include/vision_msgs/vision_msgs
)

install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

# 视频发布节点
add_executable(video_publisher 
  src/nodes/video_publisher.cpp
)
target_link_libraries(video_publisher
  ${rclcpp_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
)
install(TARGETS video_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# YOLO检测节点
add_executable(yolo_detector 
  src/nodes/yolo_detector.cpp
)
target_link_libraries(yolo_detector
  ${rclcpp_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${OpenCV_LIBS}
  ${vision_msgs_LIBRARIES}
  cv_bridge::cv_bridge
)
install(TARGETS yolo_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()