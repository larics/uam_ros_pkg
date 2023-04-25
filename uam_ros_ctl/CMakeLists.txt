cmake_minimum_required(VERSION 3.0.2)
project(uam_ros_ctl)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    roscpp
    std_msgs
    tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_move_group
        moveit_msgs
        roscpp 
        std_msgs
        tf
    DEPENDS 
        Eigen3 
        message_runtime  
)

include_directories(
    include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

#########
# Build #
#########
set(PROJECT_LIB_FILES
    src/uam_ros_ctl.cpp
)

## Declare a C++ library
add_library(
        ${PROJECT_NAME}_lib
        ${PROJECT_LIB_FILES}
)

add_executable(
        ${PROJECT_NAME}_node
        src/uam_ros_ctl_node.cpp
)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_lib
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
)

