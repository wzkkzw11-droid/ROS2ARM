cmake_minimum_required(VERSION 3.8)
project(arm_commander_c)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(arm_interface REQUIRED)
find_package(cm REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface example_interfaces arm_interface cm)

add_executable(commander2 src/commander2.cpp)
ament_target_dependencies(commander2 rclcpp moveit_ros_planning_interface example_interfaces arm_interface cm)

install(TARGETS
  test_moveit
  commander
  commander2
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()