#ifndef UAM_ROS_LITTER_COLLECT_H
#define UAM_ROS_LITTER_COLLECT_H

#include <iostream>
#include "stdio.h"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

// Conversions 
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// Eigen
#include <Eigen/Dense>







class UamRosLitterCtl
{
    public: 
        UamRosLitterCtl(ros::NodeHandle nh);
        ~UamRosLitterCtl();

        void run(); 

    private: 
        
        // ROS node handle
        ros::NodeHandle nodeHandle_; 
        ros::NodeHandle nodeHandleWithoutNs_;  

        // ROS Publishers 
        ros::Publisher m_pubTrajectoryCmd;

        // ROS Subscribers 
        ros::Subscriber m_subTargetPose;

        bool initPublishers();
        bool initSubscribers();

        void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
        trajectory_msgs::MultiDOFJointTrajectory planQuadraticBezierCurve(const geometry_msgs::Point start_point, 
                                                                           const geometry_msgs::Point end_point, 
                                                                           const geometry_msgs::Point control_point, 
                                                                           const double time_step);

        bool trashLocalized = false;

}; 

#endif // UAM_ROS_LITTER_COLLECT_H