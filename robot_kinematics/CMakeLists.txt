cmake_minimum_required(VERSION 3.8)
project(robot_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(SQLite3 REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)
add_executable(robot_state_get src/robot_state_get.cpp)
ament_target_dependencies(robot_state_get geometry_msgs std_msgs rclcpp moveit_ros_planning_interface moveit_ros_move_group tf2_ros robot_interfaces)

add_executable(robot_key_controller src/robot_key_controller.cpp)
ament_target_dependencies(robot_key_controller 
    rclcpp 
    moveit_ros_planning_interface 
    moveit_ros_move_group 
    moveit_msgs 
    geometry_msgs 
    std_msgs 
    trajectory_msgs 
    robot_interfaces 
    sensor_msgs 
    tf2_ros 
    tf2_geometry_msgs
    rosbag2_cpp
    rosbag2_storage
    rosbag2_transport
    ament_index_cpp
    SQLite3
)

add_executable(robot_func src/robot_func.cpp)
ament_target_dependencies(robot_func 
    geometry_msgs
    std_msgs 
    rclcpp 
    moveit_ros_planning_interface 
    moveit_ros_move_group 
    rclcpp_action 
    actionlib_msgs 
    sensor_msgs 
    trajectory_msgs
    control_msgs
    robot_interfaces 
)

install(TARGETS
    robot_state_get
    robot_func
    robot_key_controller
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
