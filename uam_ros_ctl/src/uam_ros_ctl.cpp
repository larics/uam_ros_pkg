#include "uam_ros_ctl.hpp"

UamRosCtl::UamRosCtl(ros::NodeHandle nh): nodeHandle_(nh)
{
    ROS_INFO_NAMED("uam_ros_ctl", "Initializing!");
    
    initPublishers(); 
    initSubscribers(); 
    setMoveGroup(); 
}


UamRosCtl::~UamRosCtl()
{
    ROS_INFO_NAMED("uam_ros_ctl", "Terminating!");
}

bool UamRosCtl::initPublishers()
{
    ROS_INFO_NAMED("uam_ros_ctl", "Initializing Publishers!");
    // TODO: Initialize publishers
    m_pubCurrentPose = nodeHandle_.advertise<geometry_msgs::Pose>("current_pose", 1);
    return true;
}

bool UamRosCtl::initSubscribers()
{
    ROS_INFO_NAMED("uam_ros_ctl", "Initializing Subscribers!");
    //m_subTargetPose = nodeHandle_.subscribe("target_pose", 1, &UamRosCtl::targetPoseCallback, this);
    // TODO: Initialize subscribers
    return true;
}

bool UamRosCtl::setMoveGroup()
{
    ROS_INFO_NAMED("uam_ros_ctl", "Setting move group!"); 

    static const std::string groupName = "arm"; 
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(groupName);

    // Allow replanning
    m_moveGroupPtr->allowReplanning(true);

    // Get current robot arm state 
    m_currentRobotStatePtr = m_moveGroupPtr->getCurrentState();

    return true; 
}

void UamRosCtl::run() {
    ROS_INFO_NAMED("uam_ros_ctl", "Running!");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        //ROS_INFO_NAMED("uam_ros_ctl", "running...");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

