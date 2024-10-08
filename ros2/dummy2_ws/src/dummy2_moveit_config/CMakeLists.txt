# cmake_minimum_required(VERSION 3.22)
# project(dummy22_moveit_config)

# find_package(ament_cmake REQUIRED)

# ament_package()

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
#   PATTERN "setup_assistant.launch" EXCLUDE)
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})


cmake_minimum_required(VERSION 3.22)
project(dummy2_moveit_config)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  Eigen3
  # interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  # moveit_task_constructor_core
  # moveit_visual_tools
  pluginlib
  rclcpp
  rclcpp_action
  # rviz_visual_tools
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

add_executable(dummy2_servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(dummy2_servo_keyboard_input PUBLIC include)
ament_target_dependencies(dummy2_servo_keyboard_input std_msgs control_msgs rclcpp)

add_executable(dummy2_servo_cpp_interface_demo src/servo_cpp_interface_demo.cpp)
target_include_directories(dummy2_servo_cpp_interface_demo PUBLIC include)
ament_target_dependencies(dummy2_servo_cpp_interface_demo moveit_servo rclcpp)

# add_executable(dummy2_pose_sets src/dummy2_pose_sets.cpp)
# target_include_directories(dummy2_pose_sets PUBLIC include)
# ament_target_dependencies(dummy2_pose_sets moveit_msgs std_msgs control_msgs rclcpp)

add_executable(dummy2_pose_sets
    src/dummy2_pose_sets.cpp)
target_include_directories(dummy2_pose_sets
    PUBLIC include)
ament_target_dependencies(dummy2_pose_sets
    ${THIS_PACKAGE_INCLUDE_DEPENDS}  
    "moveit_visual_tools"
    Boost)


install(
  TARGETS
    dummy2_servo_keyboard_input
    dummy2_servo_cpp_interface_demo
    dummy2_pose_sets
    DESTINATION
    lib/${PROJECT_NAME}
)

# install(PROGRAMS
#   src/dummy2_pose_sets.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
