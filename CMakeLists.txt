cmake_minimum_required(VERSION 2.8.3)
project(kdl_tree_fk_tutorial)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  roscpp
  sensor_msgs
  tf2_kdl
  tf2_ros
  kdl_parser
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(my_robot_state_publisher src/my_robot_state_publisher.cpp)
add_dependencies(my_robot_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_robot_state_publisher ${catkin_LIBRARIES})
