#include "uam_ros_litter_ctl.hpp"

UamRosLitterCtl::UamRosLitterCtl(ros::NodeHandle nh): nodeHandle_(nh)
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Initializing!");
    
    initPub = initPublishers(); 
    initSub = initSubscribers(); 
    initSrv = initServices();

    ros::Duration(5.0).sleep();
    if (initPub && initSub){uavState = INIT;}
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Initialized!");
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
    m_subTracker = nodeHandleWithoutNs_.subscribe("/red/tracker/input_trajectory", 1, &UamRosLitterCtl::trackerCb, this);
    // TODO: Initialize subscribers
    return true;
}

bool UamRosLitterCtl::initServices()
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Initializing Services!");
    m_readySrv = nodeHandle_.advertiseService("ready", &UamRosLitterCtl::readySrvCb, this);
    return true; 
}

bool UamRosLitterCtl::readySrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Ready service callback!");
    //uavState = READY; --> set to search so we can call bezier ASAP on target
    uavState = SEARCH; 
    return true;
}

void UamRosLitterCtl::targetPoseCb(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Target pose callback!");
    trashLocalized = true;
    targetPose.position = msg->position;
    targetPose.orientation = msg->orientation;
}

void UamRosLitterCtl::currentPoseCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    poseReciv = true; 
    currentPose.header = msg->header;
    currentPose.pose = msg->pose.pose;
    currentVel.linear = msg->twist.twist.linear;
    currentVel.angular = msg->twist.twist.angular;

}

void UamRosLitterCtl::trackerCb(const trajectory_msgs::MultiDOFJointTrajectory trajectory){
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Tracker callback!"); 
    if (trajectory.points.size() > 0 && firstTrajSent) {
        trajReciv = true;
    }
    else {
        trajReciv = false;
    }
}

float UamRosLitterCtl::switchVal(float val){
    if (std::abs(1-val) < 1e-3) return 1; 
    else return 0;
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


trajectory_msgs::MultiDOFJointTrajectory UamRosLitterCtl::resampleAndMergeTrajectories(const trajectory_msgs::MultiDOFJointTrajectory& traj1, 
                                                                                       const trajectory_msgs::MultiDOFJointTrajectory& traj2)
{
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Resampling and merging trajectories!");
    trajectory_msgs::MultiDOFJointTrajectory trajectoryCmd;
    for(int i = 0; i < traj1.points.size(); i++){
        trajectoryCmd.points.push_back(traj1.points[i]);
    }
    // Remove first point!
    for(int i= 1; i < traj2.points.size(); i++){
        trajectoryCmd.points.push_back(traj2.points[i]);
    }

    return trajectoryCmd;
}
trajectory_msgs::MultiDOFJointTrajectory UamRosLitterCtl::planLawnmowerTrajectory( const float length,
                                                                                   const float width,   
                                                                                   const double step_size)
{
    //TODO: Add parametrizable planning of the lawnmower trajectory (take yaw in consideration)
    trajectory_msgs::MultiDOFJointTrajectory trajectory; 
    int area = static_cast<int>(length*width);
    int width_ = static_cast<int>(width);
    int N = static_cast<int>(area/step_size);
    double wStep = area/length;
    wStep = width_/wStep;

    lastX = currentPose.pose.position.x; lastY = currentPose.pose.position.y;
    lastRotZ = currentPose.pose.orientation.z; lastRotW = currentPose.pose.orientation.w;

    // TODO: Generalize!
    for (int i = 0; i < N; i++){

        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        point.transforms.resize(1);
        double lawnStep = i * step_size; 
        point.transforms[0].translation.x += lastX + lawnmowerDir * step_size;
        point.transforms[0].translation.y = lastY;

        if (std::fmod(lawnStep, width_) == 0.0 && i != 0){ 
            point.transforms[0].translation.y += wStep;
            lawnmowerDir *= -1;
            // Switch Z and W for rotation 
            lastRotW = switchVal(lastRotW);
            lastRotZ = switchVal(lastRotZ);
            //TODO: Add orientation change 
        }
        point.transforms[0].translation.z = currentPose.pose.position.z;
        point.transforms[0].rotation.x = 0;
        point.transforms[0].rotation.y = 0;
        point.transforms[0].rotation.z = lastRotZ;
        point.transforms[0].rotation.w = lastRotW; 

        point.time_from_start = ros::Duration(i*step_size);
        trajectory.points.push_back(point);
        // lastX, lastY
        lastX = point.transforms[0].translation.x;
        lastY = point.transforms[0].translation.y;  

    }

    return trajectory; 

}


void UamRosLitterCtl::run() {
    ROS_INFO_NAMED("uam_ros_litter_ctl", "Running!");
    ros::Rate loop_rate(10);
    while (ros::ok())
    {   
        trajectory_msgs::MultiDOFJointTrajectory trajectoryCmd; 
        if (uavState == READY){
            //TODO: Add search state 
            double step_size = 0.5; 
            // TODO: Add current yaw to the trajectory
            trajectoryCmd = planLawnmowerTrajectory(10, 10, step_size);
            m_pubTrajectoryCmd.publish(trajectoryCmd);
            if (trajReciv = true) uavState = SEARCH;
        }

        // TODO: REFACTOR THIS!
        // TODO: Think of a state machine control (Maybe there's even something implemented))
        if (trashLocalized && uavState == SEARCH) {
            
            // TODO: For uav grab current pose 0
            // TODO: For cp grab pose without height
            // TODO: For final pose grab pose of an localized object
            //pts_ = {"uav": (9, 8, 3), 
            //"cp": (9, 8, 1), 
            //"goal": (3, 8, 1)}

            geometry_msgs::Point start_point, control_point1, goal_point;
            start_point.x = currentPose.pose.position.x;
            start_point.y = currentPose.pose.position.y;
            start_point.z = currentPose.pose.position.z;
            control_point1.x = currentPose.pose.position.x;
            control_point1.y = currentPose.pose.position.y;
            control_point1.z = 0.75;
            goal_point.x = targetPose.position.x;
            goal_point.y = targetPose.position.y;
            goal_point.z = targetPose.position.z;

            float time_step = 0.05;
            bool single = false; 
            if (single){
                trajectoryCmd = planQuadraticBezierCurve(start_point, control_point1, goal_point, time_step);
                m_pubTrajectoryCmd.publish(trajectoryCmd);
            }else{
                geometry_msgs::Point control_point2, end_point;
                trajectory_msgs::MultiDOFJointTrajectory traj1, traj2, trajComplete; 
                float x_dist = std::abs(start_point.x - goal_point.x);
                float y_dist = std::abs(start_point.y - goal_point.y);
                control_point2.x = goal_point.x + x_dist;
                control_point2.y = goal_point.y + y_dist;
                control_point2.z = 0.75;
                end_point.x = control_point2.x;
                end_point.y = control_point2.y;
                end_point.z = start_point.z;

                traj1 = planQuadraticBezierCurve(start_point, control_point1, goal_point, time_step);
                traj2 = planQuadraticBezierCurve(goal_point, control_point2, end_point, time_step);
                trajComplete = resampleAndMergeTrajectories(traj1, traj2);
                ROS_INFO_STREAM("Full trajectory: " << trajComplete); 
                m_pubTrajectoryCmd.publish(trajComplete);                 
                
            }
            
            
            start_time = ros::Time::now().toSec();
            uavState = PICKUP; 
        }

        int timeout = 3.0; 
        int elapsed = ros::Time::now().toSec() - start_time;
        if (!pickUpComplete && elapsed > timeout && uavState == PICKUP){
            if (std::abs(currentVel.linear.x) < 0.1 && std::abs(currentVel.linear.y) < 0.1 && std::abs(currentVel.linear.z)< 0.05)
            {
                ROS_INFO_NAMED("uam_ros_litter_ctl", "Pickup complete!"); 
                pickUpComplete = true;
            }
            else
            {
                ROS_INFO("Current velocity: [%f, %f, %f]", currentVel.linear.x, currentVel.linear.y, currentVel.linear.z);
            }
            uavState = PICKUP_COMPLETE;
        }
        /*

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
        */

        ros::spinOnce();
        loop_rate.sleep();
    }
}

