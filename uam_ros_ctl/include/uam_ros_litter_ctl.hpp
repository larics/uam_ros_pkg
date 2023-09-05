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
#include <std_srvs/Trigger.h>

// Conversions 
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// Eigen
#include <Eigen/Dense>


// TODO: Think of the switches based on the current state
enum State
{
    //TODO: Maybe correlate with the mavros states
    NOT_INIT = 0,
    INIT=1,
    READY=2,
    SEARCH=3,
    PICKUP=4,
    DROPOFF=5,
    LAND=6
};



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
        ros::Subscriber m_subTracker;

        // ROS Services
        ros::ServiceServer m_readySrv; 

        // init methods
        bool initPublishers();
        bool initSubscribers();
        bool initServices(); 

        // callbacks 
        void targetPoseCb(const geometry_msgs::Pose::ConstPtr& msg);
        void currentPoseCb(const nav_msgs::Odometry::ConstPtr& msg);
        void trackerCb(const trajectory_msgs::MultiDOFJointTrajectory trajectory);

        bool readySrvCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // plan curves methods 
        trajectory_msgs::MultiDOFJointTrajectory planQuadraticBezierCurve(const geometry_msgs::Point start_point, 
                                                                           const geometry_msgs::Point end_point, 
                                                                           const geometry_msgs::Point control_point, 
                                                                           const double time_step);

        trajectory_msgs::MultiDOFJointTrajectory planLawnmowerTrajectory(const float length,
                                                                         const float width,   
                                                                         const double step_size);

        trajectory_msgs::MultiDOFJointTrajectory planLinearBezierCurve(const geometry_msgs::Point end_point, 
                                                                       const double time_step); 

        // init flags
        bool initPub = false;
        bool initSub = false; 
        bool initSrv = false; 
        // CTL flags
        bool trashLocalized = false;
        bool poseReciv = false;
        bool trajReciv = false; 
        bool pickUpComplete = false;
        bool firstTrajSent = false;
        // lawn mower trajectory flags
        int lawnmowerDir = 1; 
        int w_i = 0;
        double lastX, lastY;

        int start_time; 

        // UAV pose
        geometry_msgs::PoseStamped currentPose;
        geometry_msgs::PoseStamped targetPose; 
        geometry_msgs::Twist currentVel; 
        
        // Initialize starting state
        State uavState = NOT_INIT; 

}; 

#endif // UAM_ROS_LITTER_COLLECT_H