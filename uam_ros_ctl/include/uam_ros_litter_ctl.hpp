#ifndef UAM_ROS_LITTER_COLLECT_H
#define UAM_ROS_LITTER_COLLECT_H

#include <iostream>
#include "stdio.h"

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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
        
        // ROS node handles
        ros::NodeHandle nodeHandle_; 
        ros::NodeHandle nodeHandleWithoutNs_;  

        // ROS Publishers 
        ros::Publisher m_pubTrajectoryCmd;
        ros::Publisher m_pubPoseCmd;

        // ROS Subscribers 
        ros::Subscriber m_subTargetPose;
        ros::Subscriber m_subCurrentPose; 

        // init methods
        bool initPublishers();
        bool initSubscribers();

        // callbacks 
        void targetPoseCb(const geometry_msgs::Pose::ConstPtr& msg);
        void currentPoseCb(const nav_msgs::Odometry::ConstPtr& msg);

        // plan curves methods 
        trajectory_msgs::MultiDOFJointTrajectory planQuadraticBezierCurve(const geometry_msgs::Point start_point, 
                                                                           const geometry_msgs::Point end_point, 
                                                                           const geometry_msgs::Point control_point, 
                                                                           const double time_step);
        trajectory_msgs::MultiDOFJointTrajectory planLinearBezierCurve(const geometry_msgs::Point end_point, 
                                                                       const double time_step); 

        // CTL flags
        bool trashLocalized = false;
        bool poseReciv = false;
        bool pickUpComplete = false; 
        int start_time; 

        // UAV pose
        geometry_msgs::PoseStamped currentPose;
        geometry_msgs::PoseStamped targetPose; 
        geometry_msgs::Twist currentVel; 

}; 

#endif // UAM_ROS_LITTER_COLLECT_H