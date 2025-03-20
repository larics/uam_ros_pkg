cmake_minimum_required(VERSION 3.0.2)
project(uam_ros_ctl)

## Compile as C++11, YAML-CPP REQ
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    moveit_servo
    trajectory_msgs
    roscpp
    std_msgs
    tf
    eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

## Must be called before catkin_package
add_service_files(
        DIRECTORY srv
        FILES
        getIk.srv
        changeState.srv
)

generate_messages(
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include srv
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
        moveit_servo
        trajectory_msgs
        roscpp 
        std_msgs
        tf
        eigen_conversions
    DEPENDS 
        Eigen3 
        yaml-cpp
)

include_directories(
    include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

#########
# Build #
#########
set(PROJECT_LIB_FILES
    src/uam_ros_ctl.cpp
    src/uam_arm_ctl.cpp
    src/uam_ros_litter_ctl.cpp
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
        yaml-cpp
)

add_executable(
    uam_ros_litter_ctl_node
    src/uam_ros_litter_ctl_node.cpp
)

target_link_libraries(
    uam_ros_litter_ctl_node
    uam_ros_ctl_lib
    ${EIGEN3_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(
    uam_arm_ctl_node
    src/uam_arm_ctl_node.cpp
)

target_link_libraries(
    uam_arm_ctl_node
    uam_ros_ctl_lib
    ${EIGEN3_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)


