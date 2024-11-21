# uam_ros_pkg

ROS pkg for aerial manipulators used on `kopterworx` UAV. 

Currently configured for the `ASAP` manipulator which has quite simple configuration. 

## How to start real robot? 

You can start real robot by running following commands: 

Run real robot (motor controllers) with: 
```
roslaunch dynamixel_workbench_controllers arm_dynamixel_controllers.launch
```

Run moveit without gazebo as follows: 
```
roslaunch uam_moveit_config demo_gazebo.launch real_robot:="true" gazebo:="false"
```

Run correct moveit as follows: 
```
roslaunch uam_moveit_config demo.launch 
```

After running kopterworx with the aerial manipulator from `ardupilot_gazebo` package. 

### Comments

If you want to edit spawn pose of the robot manipulator, edit `demo_gazebo.launch` arguments. 

### Deps

Currently docker misses some dependencies so it is neccessary to install them by hand as follows: 
```
sudo apt-get install ros-noetic-topic-tools \
	ros-noetic-controller-manager \
	ros-noetic-rviz \
	ros-noetic-moveit \
	ros-noetic-moveit-simple-controller-manager
```

After that run: 
`source /opt/ros/noetic/setup.bash
cd /root/catkin_ws
catkin build
`

And then you can run 
```
roslaunch uam_ros_pkg uam_ros_ctl.launch
```


## TODO: 

- [x] Enable simple control of the real manipulator (remapping moveit trajectory) 
- [x] Write simple control node for the manipulator in the ROS pkg `uam_ctl`
- [x] Add joint limits 
- [x] Change planner 
- [x] Test moveit movement
- [x] Enable `control_arm_node` 
- [x] Publish end effector position on the topic
- [x] Added topic relay for the moveit (state publishing) 
- [ ] Test with different planning frame on the UAV 
- [ ] Test with moveit servo 
- [ ] Filter out topics that are needed, and ones that are not!
- [ ] Enable joint commands
- [ ] Add functionality for simple trajectory execution
- [ ] Test on real UAM (real manipulator) 
- [ ] REFACTOR EVERYTHING! 
