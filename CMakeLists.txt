cmake_minimum_required(VERSION 3.8)
project(group31_assignment_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)               # ajout pour /joint_states
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  # pour MoveIt2

# --- Executables ---

# Node AprilTag Detector
add_executable(apriltag_detector src/apriltag_detector.cpp)
ament_target_dependencies(apriltag_detector rclcpp std_msgs geometry_msgs apriltag_msgs)

# Node Cube Pose Transformer
add_executable(cube_pose_transformer src/cube_pose_transformer.cpp)
ament_target_dependencies(cube_pose_transformer rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)

# Node Move Arm (avec récupération joint_states)
add_executable(move_arm src/move_arm.cpp)
ament_target_dependencies(move_arm rclcpp geometry_msgs moveit_ros_planning_interface sensor_msgs)

# --- Installation ---
install(TARGETS
  apriltag_detector
  cube_pose_transformer
  move_arm
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

