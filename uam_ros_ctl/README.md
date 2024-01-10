## uam_ros_ctl

High level control for aerial manipulators. 

Current package consists of following nodes: 
* `uam_arm_ctl` 
* `uam_litter_ctl`

Collect litter with the Bezier curve (quadratic one, for this case). 

### Litter planning node

Run node with: 
```
roslaunch uam_ros_litter litter_ctl.launch 
```

### UAM arm ctl node 

Run node for the arm control with: 

```
roslaunch uam_ros_ctl uam_arm_ctl.launch 
```

### Branches

- Added new branch [arm-dev] to concentrate on enabling following functionalities of the stand-alone robot arm: 
* normal control with `control_arm_node` wrapper 
* enabling moveit_servo as part of the code or the state control 
* enabling it to work in simulation 
* connecting with the real arm 
* connecting it with the UAV in simulation to get UAM 

### Controllers

One of the biggest challanges of this work is how to connect it with the existing UAV controllers and how to 
specify robots in correct namespacing to enable spawning of multiple aerial manipulators. 

Here we can add interesting links: 
- [Namespacing with controller_manager](https://answers.ros.org/question/264359/starting-a-controller-in-the-same-namespace-as-the-controller_manager/) 

### TODO: 
- [litter_ctl] 
- [ ] Add config for the topic names 
- [ ] Add states || conditions for the pickup 
- [ ] Test with the movable object + instance segmentation 
- [arm_ctl] 
- [x] enable control_arm_node
- [ ] fix controllers 
- [ ] connect with moveit 
- [ ] plan and execute path
- [ ] enable moveit servo
- [ ] connect controllers with the UAV [ardupilot_gazebo](https://github.com/larics/ardupilot_gazebo/tree/fzoric/devel) 

