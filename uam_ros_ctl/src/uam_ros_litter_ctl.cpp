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
    m_pubTrajectoryCmd = nodeHandleWithoutNs_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("red/tracker/input_trajectory", 100);
    m_pubPoseCmd = nodeHandleWithoutNs_.advertise<geometry_msgs::PoseStamped>("red/tracker/input_pose", 100);
    return true;
}

bool UamRosLitterCtl::initSubscribers()
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Initializing Subscribers!");
    m_subTargetPose = nodeHandle_.subscribe("target_pose", 1, &UamRosLitterCtl::targetPoseCb, this);
    m_subCurrentPose = nodeHandleWithoutNs_.subscribe("/red/mavros/local_position/odom", 1, &UamRosLitterCtl::currentPoseCb, this);
    // TODO: Initialize subscribers
    return true;
}

void UamRosLitterCtl::targetPoseCb(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Target pose callback!");
    trashLocalized = true;
}

void UamRosLitterCtl::currentPoseCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    poseReciv = true; 
    currentPose.header = msg->header;
    currentPose.pose = msg->pose.pose;
    currentVel.linear = msg->twist.twist.linear;
    currentVel.angular = msg->twist.twist.angular;

}


trajectory_msgs::MultiDOFJointTrajectory UamRosLitterCtl::planQuadraticBezierCurve(const geometry_msgs::Point start_point,
                                                                                   const geometry_msgs::Point control_point, 
                                                                                   const geometry_msgs::Point end_point,  
                                                                                   const double time_step)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory; 
    //Eigen::MatrixXd BezierQuad;
    double scale_factor = 2.0; 

    //TODO: Add planning of a Bezier Curve
    Eigen::Vector3d p0(start_point.x, start_point.y, start_point.z);
    Eigen::Vector3d p1(control_point.x, control_point.y, control_point.z);
    Eigen::Vector3d p2(end_point.x, end_point.y, end_point.z);

    // Set initial parametrized time
    Eigen::VectorXd t;
    int N = static_cast<int>(1/time_step+1); 
    t.setLinSpaced(N, 0, 1);
    Eigen::VectorXd ones = Eigen::VectorXd::Ones( t.rows( ) );

    // Compute Bezier Curve
    // TODO: Check how to make this nicer
    // https://medium.com/geekculture/2d-and-3d-b%C3%A9zier-curves-in-c-499093ef45a9
    // https://blog.demofox.org/2016/03/05/matrix-form-of-bezier-curves/
    //Bquad = ((1-t)**2)*p0 + 2*(1-t)*t*p1 + t**2*p2
    Eigen::VectorXd t1 = (ones-t).array().pow(2);
    Eigen::VectorXd t2 = (ones-t).matrix().asDiagonal()*t; 
    Eigen::VectorXd t3 = t.array().pow(2); 
    Eigen::MatrixXd BezierQuad = t1*p0.transpose() + 2*t2*p1.transpose() + t3*p2.transpose();

    for (int i = 0; i < t.rows(); i++) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        point.transforms.resize(1);
        point.transforms[0].translation.x = BezierQuad(i, 0);
        point.transforms[0].translation.y = BezierQuad(i, 1);
        point.transforms[0].translation.z = BezierQuad(i, 2);
        point.transforms[0].rotation.x = 0;
        point.transforms[0].rotation.y = 0;
        point.transforms[0].rotation.z = 0;
        point.transforms[0].rotation.w = 1;
        point.time_from_start = ros::Duration(i*time_step*scale_factor);
        trajectory.points.push_back(point);
    }
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Planning quadratic bezier curve!");

    return trajectory; 
}


void UamRosLitterCtl::run() {
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Running!");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        // TODO: REFACTOR THIS!
        // TODO: Think of a state machine control (Maybe there's even something implemented)
        if (trashLocalized && poseReciv) {
            
            // TODO: For uav grab current pose 0
            // TODO: For cp grab pose without height
            // TODO: For final pose grab pose of an localized object
            //pts_ = {"uav": (9, 8, 3), 
            //"cp": (9, 8, 1), 
            //"goal": (3, 8, 1)}

            trajectory_msgs::MultiDOFJointTrajectory trajectoryCmd; 

            geometry_msgs::Point start_point, control_point, goal_point;
            start_point.x = currentPose.pose.position.x;
            start_point.y = currentPose.pose.position.y;
            start_point.z = currentPose.pose.position.z;
            control_point.x = currentPose.pose.position.x;
            control_point.y = currentPose.pose.position.y;
            control_point.z = 0;
            goal_point.x = targetPose.pose.position.x;
            goal_point.y = targetPose.pose.position.y;
            goal_point.z = 1;

            ROS_INFO_NAMED("uam_ros_litter_ctl", "Publishing trajectory command!");
            trajectoryCmd = planQuadraticBezierCurve(start_point, control_point, goal_point, 0.05);       
            m_pubTrajectoryCmd.publish(trajectoryCmd);  
            start_time = ros::Time::now().toSec();
            trashLocalized = false;
        }

        int timeout = 3.0; 
        int elapsed = ros::Time::now().toSec() - start_time;
        if (!pickUpComplete && elapsed > timeout){
            if (std::abs(currentVel.linear.x) < 0.05 && std::abs(currentVel.linear.y) < 0.05 && std::abs(currentVel.linear.z)< 0.05)
            {
                ROS_INFO_NAMED("uam_ros_litter_ctl", "Pickup complete!"); 
                pickUpComplete = true;
            }
            else
            {
                ROS_INFO("Current velocity: [%f, %f, %f]", currentVel.linear.x, currentVel.linear.y, currentVel.linear.z);
            }
        }

        if (pickUpComplete){

            ROS_INFO_NAMED("uam_ros_litter", "Pickup complete!"); 
            geometry_msgs::PoseStamped poseCmd; float heightAdd = 5.0; 
            poseCmd.pose.position.x = currentPose.pose.position.x;
            poseCmd.pose.position.y = currentPose.pose.position.y; 
            poseCmd.pose.position.z = currentPose.pose.position.z + heightAdd;
            poseCmd.pose.orientation = currentPose.pose.orientation;  
            m_pubPoseCmd.publish(poseCmd); 
            pickUpComplete = false;
            trashLocalized = false; 
        }
        //ROS_INFO_NAMED("uam_ros_ctl", "running...");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

