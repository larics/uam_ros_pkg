#include <ros/ros.h>
#include "uam_ros_litter_ctl.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "uam_ros_litter_ctl");
    ros::NodeHandle nodeHandle("~"); 
    UamRosLitterCtl UamRosLitterCtl(nodeHandle); 

    int num_threads = 2;     
    ros::AsyncSpinner spinner(num_threads);
    spinner.start(); 
    UamRosLitterCtl.run(); 

    return 0; 

}