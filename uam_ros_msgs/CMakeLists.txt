cmake_minimum_required(VERSION 3.0.2)
project(uam_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs  
)

## Declare message and service files
#set(MSG_FILES
  # Add message files
#)

set(SRV_FILES
  "changeState.srv" 
  "getIk.srv"
)

## Generate added messages and services
#add_message_files(
#  FILES msg
#  ${MSG_FILES}
#)

add_service_files(
  DIRECTORY srv
  FILES
  ${SRV_FILES}
)

## Generate messages and services
generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
  sensor_msgs  
)

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

## Install rules (optional)
install(DIRECTORY msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)