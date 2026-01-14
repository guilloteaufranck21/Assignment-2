cmake_minimum_required(VERSION 3.8)
project(assignment)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(apriltag_detector src/apriltag_detector.cpp)

ament_target_dependencies(apriltag_detector
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(tag_transformer src/tag_transformer.cpp)

ament_target_dependencies(tag_transformer 
  rclcpp 
  geometry_msgs 
  tf2_ros 
  tf2_geometry_msgs
)
target_link_libraries(apriltag_detector 
  ${OpenCV_LIBRARIES} 
  apriltag::apriltag
)

install(TARGETS
  apriltag_detector
  tag_transformer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()