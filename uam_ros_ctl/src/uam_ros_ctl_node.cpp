#include <ros/ros.h>
#include "uam_ros_ctl.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "uam_ros_ctl");
    ros::NodeHandle nodeHandle("~"); 
    UamRosCtl uamRosCtl(nodeHandle); 

    int num_threads = 6;     
    ros::AsyncSpinner spinner(num_threads);
    spinner.start(); 
    uamRosCtl.run(); 

    return 0; 

}