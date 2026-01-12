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
find_package(sensor_msgs REQUIRED) 
find_package(apriltag_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# --- Executables ---

# Node Move Arm (avec récupération joint_states)
add_executable(move_arm src/move_arm.cpp)
ament_target_dependencies(move_arm rclcpp geometry_msgs moveit_ros_planning_interface sensor_msgs)

# --- Installation ---
install(TARGETS
  move_arm
  DESTINATION lib/${PROJECT_NAME}
)

# --- Launch files ---
install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
)

# --- Tests ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

