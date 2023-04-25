#ifndef UAM_ROS_CTL_H
#define UAM_ROS_CTL_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>

// Conversions 
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>



class UamRosCtl
{
    public: 
        UamRosCtl(ros::NodeHandle& nh);
        ~UamRosCtl();

    private: 
        
        // ROS node handle
        ros::NodeHandle nH_; 
        ros::NodeHandle nHWithoutNs_; 
        
        moveit::planning_interface::MoveGroupInterfaceConstPtr m_moveGroupPtr;
        robot_state::JointModelGroupConstPtr                   m_jointModelGroupPtr; 
        planning_scene::PlanningSceneConstPtr                  m_planningScenePtr; 
        moveit::core::RobotStatePtr                            m_currentRobotStatePtr; 

        // ROS Publishers 
        ros::Publisher m_pubCurrentPose;
        ros::Publisher m_pubCurrentJointState;

        // ROS Subscribers 
        ros::Subscriber m_subTargetPose;
        ros::Subscriber m_subTargetJointState;
        ros::Subscriber m_subDeltaTargetPose;

        bool initPublishers();
        bool initSubscribers();

}; 


#endif // UAM_ROS_CTL_H