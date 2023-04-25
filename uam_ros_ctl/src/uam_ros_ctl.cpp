#include "uam_ros_ctl.hpp"

UamRosCtl::UamRosCtl(ros::NodeHandle nh): nodeHandle_(nh)
{
    ROS_INFO_NAMED("uam_ros_ctl", "Initializing!");
}


UamRosCtl::~UamRosCtl()
{
    ROS_INFO_NAMED("uam_ros_ctl", "Terminating!");
}

bool UamRosCtl::initPublishers()
{
    ROS_INFO_NAMED("uam_ros_ctl", "Initializing Publishers!");
    // TODO: Initialize publishers
    return true;
}

bool UamRosCtl::initSubscribers()
{
    ROS_INFO_NAMED("uam_ros_ctl", "Initializing Subscribers!");
    // TODO: Initialize subscribers
    return true;
}

void UamRosCtl::run() {
    ROS_INFO_NAMED("uam_ros_ctl", "Running!");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

