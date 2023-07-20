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
                                                                                   const geometry_msgs::Point control_point, 
                                                                                   const geometry_msgs::Point end_point,  
                                                                                   const double time_step)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory; 
    //Eigen::MatrixXd BezierQuad;
    double scale_factor = 5.0; 

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

        // TODO: Think of a state machine control (Maybe there's even something implemented)
        if (trashLocalized) {
            
            // TODO: For uav grab current pose 0
            // TODO: For cp grab pose without height
            // TODO: For final pose grab pose of an localized object
            //pts_ = {"uav": (9, 8, 3), 
            //"cp": (9, 8, 1), 
            //"goal": (3, 8, 1)}

            trajectory_msgs::MultiDOFJointTrajectory trajectoryCmd; 

            geometry_msgs::Point start_point, control_point, goal_point;
            start_point.x = 9; start_point.y = 4; start_point.z = 5;
            control_point.x = 9; control_point.y = 4; control_point.z = -1;
            goal_point.x = 1; goal_point.y = 4; goal_point.z = 2;

            ROS_INFO_NAMED("uam_ros_litter_ctl", "Publishing trajectory command!");
            trajectoryCmd = planQuadraticBezierCurve(start_point, control_point, goal_point, 0.25);   
            std::cout << "TrajectoryCmd: " << trajectoryCmd << std::endl;    
            m_pubTrajectoryCmd.publish(trajectoryCmd); 
            trashLocalized = false;    
        
        }
        //ROS_INFO_NAMED("uam_ros_ctl", "running...");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

