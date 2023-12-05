#include <ros/ros.h>
#include "uam_arm_ctl.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_arm");
    ros::NodeHandle nodeHandle("~"); 
    ControlArm controlArm(nodeHandle); 

    int num_threads = 4;     
    ros::AsyncSpinner spinner(num_threads);
    spinner.start(); 
    controlArm.run(); 

    return 0; 

}