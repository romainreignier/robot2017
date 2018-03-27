cmake_minimum_required(VERSION 2.8.3)
project(robot_move)

set(CMAKE_BUILD_TYPE Debug)
## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dwb_msgs
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav_core2
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf
  dwb_local_planner
  message_generation
)

add_service_files(
   FILES
   FollowSegment.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs nav_2d_msgs # Or other packages containing msgs
)

catkin_package(
    LIBRARIES dwb_local_planner debug_dwb_local_planner
    CATKIN_DEPENDS
        costmap_2d dwb_msgs geometry_msgs nav_2d_msgs nav_2d_utils nav_core2
        nav_msgs pluginlib roscpp sensor_msgs tf dwb_local_planner
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_move src/robot_move.cpp)
add_dependencies(robot_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_move ${catkin_LIBRARIES})

