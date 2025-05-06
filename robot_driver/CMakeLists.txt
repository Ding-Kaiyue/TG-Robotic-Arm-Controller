cmake_minimum_required(VERSION 3.8)
project(robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(robot_driver SHARED
  src/robot_driver_cantx_new.cpp  
  src/robot_driver_canrx_new.cpp  
)

set_target_properties(robot_driver PROPERTIES
  OUTPUT_NAME "robot_driver"  
  VERSION 1.0.0
  SOVERSION 1
)

ament_target_dependencies(robot_driver
  rclcpp
  std_msgs
  can_msgs
  ros2_socketcan
  sensor_msgs
  robot_interfaces
)

add_executable(robot_driver_node src/main.cpp) 
target_link_libraries(robot_driver_node
  robot_driver 
)
ament_target_dependencies(robot_driver_node
  rclcpp
)

include_directories(
  include
  ${ros2_socketcan_INCLUDE_DIRS}
  ${robot_interfaces_INCLUDE_DIRS}
)

install(TARGETS
  robot_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  robot_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(robot_driver)
ament_export_dependencies(
  rclcpp
  std_msgs
  can_msgs
  ros2_socketcan
  sensor_msgs
  robot_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
