cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs  
)

### BUild

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Install
install(DIRECTORY launch params scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
