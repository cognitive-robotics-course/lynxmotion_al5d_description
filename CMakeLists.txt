cmake_minimum_required(VERSION 3.0.2)
project(lynxmotion_al5d_description)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  tf2
  roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  RPY.msg
  Pose.msg
)

add_service_files(
  FILES
  SpawnBrick.srv
  KillBrick.srv
  TeleportAbsolute.srv
  TeleportRelative.srv
  Clear.srv
  Reset.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp roslib
)

add_executable(${PROJECT_NAME}_manager_node src/brick.cpp src/manager.cpp src/manager_node.cpp)
add_dependencies(${PROJECT_NAME}_manager_node ${PROJECT_NAME}_gencpp)
add_executable(${PROJECT_NAME}_spawn_brick src/spawn_brick.cpp)
add_dependencies(${PROJECT_NAME}_spawn_brick ${PROJECT_NAME}_gencpp)
add_executable(${PROJECT_NAME}_kill_brick src/kill_brick.cpp)
add_dependencies(${PROJECT_NAME}_kill_brick ${PROJECT_NAME}_gencpp)

set_target_properties(${PROJECT_NAME}_manager_node PROPERTIES OUTPUT_NAME manager_node PREFIX "")
set_target_properties(${PROJECT_NAME}_spawn_brick PROPERTIES OUTPUT_NAME spawn_brick PREFIX "")
set_target_properties(${PROJECT_NAME}_kill_brick PROPERTIES OUTPUT_NAME kill_brick PREFIX "")

target_link_libraries(${PROJECT_NAME}_manager_node
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_spawn_brick
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_kill_brick
  ${catkin_LIBRARIES}
)

