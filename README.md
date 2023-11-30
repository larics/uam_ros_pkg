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


## TODO: 

- [x] Enable simple control of the real manipulator (remapping moveit trajectory) 
- [x] Write simple control node for the manipulator in the ROS pkg `uam_ctl`
- [x] Add joint limits 
- [x] Change planner 
- [x] Test moveit movement
- [ ] Enable `control_arm_node` 
- [ ] Enable joint commands
- [ ] Publish end effector position on the topic 
- [ ] Add functionality for simple trajectory execution
- [ ] Test on real UAM (real manipulator) 
