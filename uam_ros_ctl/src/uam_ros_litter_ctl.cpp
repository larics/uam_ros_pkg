#include "uam_ros_litter_ctl.hpp"

UamRosLitterCtl::UamRosLitterCtl(ros::NodeHandle nh): nodeHandle_(nh)
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Initializing!");
    
    initPublishers(); 
    initSubscribers(); 
}


UamRosLitterCtl::~UamRosLitterCtl()
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Terminating!");
}

bool UamRosLitterCtl::initPublishers()
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Initializing Publishers!");
    // TODO: Initialize publishers
    m_pubTrajectoryCmd = nodeHandle_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("current_pose", 1);
    return true;
}

bool UamRosLitterCtl::initSubscribers()
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Initializing Subscribers!");
    m_subTargetPose = nodeHandle_.subscribe("target_pose", 1, &UamRosLitterCtl::targetPoseCallback, this);
    // TODO: Initialize subscribers
    return true;
}

void UamRosLitterCtl::targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Target pose callback!");
    trashLocalized = true;
}

trajectory_msgs::MultiDOFJointTrajectory UamRosLitterCtl::planQuadraticBezierCurve(const geometry_msgs::Point start_point, 
                                                                                   const geometry_msgs::Point end_point, 
                                                                                   const geometry_msgs::Point control_point, 
                                                                                   const double time_step)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory;

    //TODO: Add planning of a Bezier Curve
    Eigen::Vector3d p0(start_point.x, start_point.y, start_point.z);
    Eigen::Vector3d p1(end_point.x, end_point.y, end_point.z);
    Eigen::Vector3d p2(control_point.x, control_point.y, control_point.z);

    // Set initial parametrized time
    Eigen::VectorXd t; 
    t.setLinSpaced(1/time_step, 0, 1);

    //# Bezier quadratic curve
    //Bquad = ((1-t)**2)*p0 + 2*(1-t)*t*p1 + t**2*p2

    Eigen::VectorXd ones = Eigen::VectorXd::Ones( t.rows( ) );

    // Compute Bezier Curve
    Eigen::MatrixXd BezierQuad = ((ones.array()-t.array()).pow(2)).matrix()*p0.transpose();

    std::cout << "BezierQuad: " << BezierQuad << std::endl;

    ROS_INFO_NAMED("uam_ros_litter_ctl", "Planning quadratic bezier curve!");

    return trajectory; 
}


void UamRosLitterCtl::run() {
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Running!");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        if (trashLocalized) {

            
        
        }
        //ROS_INFO_NAMED("uam_ros_ctl", "running...");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

