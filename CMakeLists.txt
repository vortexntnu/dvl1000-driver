cmake_minimum_required(VERSION 2.8.3)
project(dvl1000_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  message_generation
)

## Uncomment this if you wish to use the
## custom messages that this package provides
# add_message_files(
#   FILES
#   DVL.msg
#   DVLBeam.msg
# )
# 
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   nav_msgs
# )

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  message_runtime
)

include_directories( ${catkin_INCLUDE_DIRS} )
