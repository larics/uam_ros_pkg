#include "uam_arm_ctl.hpp"

ControlArm::ControlArm(ros::NodeHandle nh) : nH(nh) {

  ROS_INFO_NAMED("arm_ctl",  "Node started.");

  enableVisualization_ = true; 
  sleepMs_ = 2000000; 

  // Initial sleep (waiting for move group and rest of the MoveIt stuff to --> Maybe there's smarter way to wait for move group?
  // initialize.)
  usleep(sleepMs_);

  // Load group and topic names
  loadConfig(); 

  // Initialize robot ctl class
  initRobot();

  // Initialize servo
  if (enable_servo) {servoPtr = initServo();}; 
  
  StatusMonitor status_monitor(nH, "servo_status");

  // Find out basic info
  getBasicInfo();
}

ControlArm::~ControlArm() {}

void ControlArm::initRobot() {

  ROS_INFO("[ControlArm] Started node initialization.");

  // Set move group and planning scene
  moveGroupInit         = setMoveGroup();
  planSceneInit         = setPlanningScene();
  ROS_INFO_NAMED("arm_ctl", "Initialized: move_group, planning_scene, planning_scene_monitor."); 

  dispTrajPub               = nH.advertise<moveit_msgs::DisplayTrajectory>(dispTrajTopicName, dispTrajQSize);
  currPosePub               = nH.advertise<geometry_msgs::Pose>(currPoseTopicName, currPoseQSize);
  cmdJointGroupPositionPub  = nHns.advertise<std_msgs::Float64MultiArray>(std::string("franka_ph/joint_group_position_controller/command"), 1);
  cmdJointGroupVelocityPub  = nHns.advertise<std_msgs::Float64MultiArray>(std::string("franka_ph/joint_group_velocity_controller/command"), 1);

  // Subscribers
  cmdPoseSub                = nH.subscribe<geometry_msgs::Pose>(cmdPoseTopicName, cmdPoseQSize, &ControlArm::cmdPoseCb, this);
  cmdToolOrientSub          = nH.subscribe<geometry_msgs::Point>(cmdToolOrientTopicName, cmdToolOrientQSize, &ControlArm::cmdToolOrientationCb, this);
  cmdDeltaPoseSub           = nH.subscribe<geometry_msgs::Pose>(cmdDeltaPoseTopicName, cmdDeltaPoseQSize, &ControlArm::cmdDeltaPoseCb, this);
  ROS_INFO_NAMED("arm_ctl", "Initialized subscribers/publishers.");

  // Initialize Services
  getIkSrv                      = nH.advertiseService(getIkSrvName, &ControlArm::getIkSrvCb, this);
  disableCollisionSrv           = nH.advertiseService(disableCollisionSrvName, &ControlArm::disableCollisionSrvCb, this);
  addCollisionObjectSrv         = nH.advertiseService(addCollisionObjectSrvName, &ControlArm::addCollisionObjectSrvCb, this);
  startPositionCtlSrv           = nH.advertiseService(startPositionCtlSrvName, &ControlArm::startPositionCtlCb, this);
  startJointTrajCtlSrv          = nH.advertiseService(startJointTrajCtlSrvName, &ControlArm::startJointTrajCtlCb, this);
  startJointGroupPositionCtlSrv = nH.advertiseService(startJointGroupPosCtlSrvName, &ControlArm::startJointGroupPositionCtlCb, this);
  startJointGroupVelocityCtlSrv = nH.advertiseService(startJointGroupVelCtlSrvName, &ControlArm::startJointGroupVelocityCtlCb, this);
  changeRobotStateSrv           = nH.advertiseService(changeRobotStateSrvName, &ControlArm::setStateCb, this); 
  ROS_INFO_NAMED("arm_ctl", "Initialized services.");

  // Initialize Clients for other services
  applyPlanningSceneSrvCli = nHns.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  applyPlanningSceneSrvCli.waitForExistence();
  ROS_INFO_NAMED("arm_ctl", "Initialized service clients. ");
}

void ControlArm::loadConfig() {

  // TODO: Add argument that's propagated through launch file
  YAML::Node config = YAML::LoadFile("/root/uav_ws/src/uam_ros_pkg/uam_ros_ctl/config/aerial_manipulator.yaml");

  // Set move group name and ee link
  GROUP_NAME = config["robot"]["arm_name"].as<std::string>(); //"panda_manipulator"; 
  EE_LINK_NAME = config["robot"]["ee_link_name"].as<std::string>(); //"panda_hand_tcp"; 
  NUM_CART_PTS = config["robot"]["num_cart_pts"].as<int>(); 

  // Topic names
  dispTrajTopicName     = config["topic"]["pub"]["display_trajectory"]["name"].as<std::string>(); 
  currPoseTopicName     = config["topic"]["pub"]["current_pose"]["name"].as<std::string>(); 
  cmdPoseTopicName      = config["topic"]["sub"]["cmd_pose"]["name"].as<std::string>(); 
  cmdDeltaPoseTopicName = config["topic"]["sub"]["cmd_delta_pose"]["name"].as<std::string>(); 

  // Q Sizes
  dispTrajQSize     = config["topic"]["pub"]["display_trajectory"]["queue"].as<int>(); 
  currPoseQSize     = config["topic"]["pub"]["current_pose"]["queue"].as<int>(); 
  cmdPoseQSize      = config["topic"]["sub"]["cmd_pose"]["queue"].as<int>(); 
  cmdDeltaPoseQSize = config["topic"]["sub"]["cmd_delta_pose"]["queue"].as<int>(); 
  ROS_INFO_NAMED("arm_ctl", "setted queue sizes!");

  // Srv names 
  disableCollisionSrvName           = config["srv"]["disable_collision"]["name"].as<std::string>(); 
  addCollisionObjectSrvName         = config["srv"]["add_collision"]["name"].as<std::string>(); 
  startPositionCtlSrvName           = config["srv"]["start_position_ctl"]["name"].as<std::string>(); 
  startJointTrajCtlSrvName          = config["srv"]["start_joint_traj_ctl"]["name"].as<std::string>(); 
  startJointGroupPosCtlSrvName      = config["srv"]["start_joint_group_pos_ctl"]["name"].as<std::string>(); 
  startJointGroupVelCtlSrvName      = config["srv"]["start_joint_group_vel_ctl"]["name"].as<std::string>(); 
  changeRobotStateSrvName           = config["srv"]["change_robot_state"]["name"].as<std::string>(); 
  ROS_INFO_NAMED("arm_ctl", "named services"); 
}

bool ControlArm::setMoveGroup() {

  ROS_INFO_NAMED("arm_ctl",  "Setting move group.");

  m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(GROUP_NAME);

  // Allow replanning
  m_moveGroupPtr->allowReplanning(true);

  // Get current robot arm state
  getArmState();

  return true;
}

bool ControlArm::setPlanningScene() {

  ROS_INFO("[ControlArm] Setting planning scene.");
  // MoveIt planning scene setup as seen
  // (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html)
  robot_model_loader::RobotModelLoader m_robotLoader("robot_description");
  robot_model::RobotModelPtr kinematic_model = m_robotLoader.getModel();
  m_planningScenePtr = new planning_scene::PlanningScene(kinematic_model);
  ROS_INFO("[ControlArm] Model frame: %s",
           kinematic_model->getModelFrame().c_str());
  return true;
}

std::unique_ptr<moveit_servo::Servo> ControlArm::initServo(){

  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM("Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Start the planning scene monitor
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  auto servo = std::make_unique<moveit_servo::Servo>(nH, planning_scene_monitor);
  
  return servo; 
}

void ControlArm::getBasicInfo() {

  if (moveGroupInit) {

    ROS_INFO("[ControlArm] Reference planning frame: %s",
             m_moveGroupPtr->getPlanningFrame().c_str());
    ROS_INFO("[ControlArm] Reference end effector frame: %s",
             m_moveGroupPtr->getEndEffectorLink().c_str());
  }
}
// This is wrong, we should pass cmdPose as argument into function and then set
// it if we plan to use setters
bool ControlArm::setCmdPose() {

  if (moveGroupInit) {
    m_moveGroupPtr->setPoseTarget(m_cmdPose);
    return true;
  }
  return false;
}

bool ControlArm::setStateCb(uam_ros_msgs::changeStateRequest &req, uam_ros_msgs::changeStateResponse &res)
{

    // Why would this be boolean? 

    auto itr = std::find(std::begin(stateNames), std::end(stateNames), req.state);
    

    if ( itr != std::end(stateNames))
    {
        int iX = std::distance(stateNames, itr); 
        robotState  = (state)iX; 
        ROS_INFO_STREAM("Switching state!");
        res.success = true;  
    }else{
        ROS_INFO_STREAM("Failed switching to state " << req.state); 
        res.success = false; 
    } 

    return res.success; 

}

void ControlArm::getArmState() {

  // method is more like refresh current kinematic state
  // (getCurrentKinematicState)
  m_currentRobotStatePtr = m_moveGroupPtr->getCurrentState();
}

void ControlArm::getEEState(const std::string eeLinkName) {

  m_endEffectorState =
      m_currentRobotStatePtr->getGlobalLinkTransform(eeLinkName);

  bool debug = false;
  if (debug) {

    ROS_INFO_STREAM("Translation: \n"
                    << m_endEffectorState.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << m_endEffectorState.rotation() << "\n");
  }
}

void ControlArm::getJointPositions(const std::vector<std::string> &jointNames, std::vector<double> &jointGroupPositions) {

  m_currentRobotStatePtr->copyJointGroupPositions(m_jointModelGroupPtr,
                                                  jointGroupPositions);

  bool debug = true;
  if (debug) {
    for (std::size_t i = 0; i < jointNames.size(); ++i) {
      ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointGroupPositions[i]);
    }
  }
}

bool ControlArm::getIK(const geometry_msgs::Pose wantedPose, const std::size_t attempts, double timeout) {

  bool found_ik = m_currentRobotStatePtr->setFromIK(m_jointModelGroupPtr, wantedPose);

  bool debug = false;
  if (debug) {
    ROS_INFO("Found IK solution!");
  }

  return found_ik;
}

bool ControlArm::getIkSrvCb(uam_ros_msgs::getIkRequest &req, uam_ros_msgs::getIkResponse &res)
{
    int attempts = 10;
    int timeout = 1;
    bool success = getIK(req.wantedPose, attempts, timeout);
    std::vector<double> jointPositions;
    m_currentRobotStatePtr->copyJointGroupPositions(m_jointModelGroupPtr,
                                                    jointPositions);

    sensor_msgs::JointState jointState;
    jointState.position = jointPositions;
    res.jointState = jointState;

    return success;
}

void ControlArm::cmdPoseCb(const geometry_msgs::Pose::ConstPtr &msg) {

  ROS_INFO("[ControlArm] Recieved cmd_pose...");

  // Set CMD pose
  m_cmdPose.position = msg->position;
  m_cmdPose.orientation = msg->orientation;

  recivPoseCmd = true; 
  
}

void ControlArm::cmdDeltaPoseCb(const geometry_msgs::Pose::ConstPtr &msg) {

  ROS_INFO("[ControlArm] Recieved cmd_delta_pose...");

  // Set delta CMD pose
  m_cmdDeltaPose.position = msg->position;
  m_cmdDeltaPose.orientation = msg->orientation;

  sendToDeltaCmdPose();
}

void ControlArm::cmdToolOrientationCb(const geometry_msgs::Point::ConstPtr &msg) {

  ROS_INFO("[ControlArm] Received cmd tool orientation...");

  // Get current end effector state
  getEEState(EE_LINK_NAME);

  geometry_msgs::Pose cmdPose;
  tf2::Quaternion cmdQuaternion;

  // Set current end effector position as command
  cmdPose.position.x = m_endEffectorState(0, 3);
  cmdPose.position.y = m_endEffectorState(1, 3);
  cmdPose.position.z = m_endEffectorState(2, 3);

  // Set commanded roll, pitch, yaw as quaternion
  cmdQuaternion.setRPY(msg->x, msg->y, msg->z);
  cmdPose.orientation.x = cmdQuaternion.x();
  cmdPose.orientation.y = cmdQuaternion.y();
  cmdPose.orientation.z = cmdQuaternion.z();
  cmdPose.orientation.w = cmdQuaternion.w();

  // set CMD pose
  m_cmdPose = cmdPose;

  recivPoseCmd = true; 
  //sendToCmdPose();
}

bool ControlArm::sendToCmdPose() {

  setCmdPose();

  // Call planner, compute plan and visualize it
  moveit::planning_interface::MoveGroupInterface::Plan plannedPath;

  // plan Path --> normal path
  bool success = (m_moveGroupPtr->plan(plannedPath) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("[ControlArm] Visualizing plan 1 (pose goal) %s",
           success ? "" : "FAILED");

  // Requires async spinner to work (Added asyncMove/non-blocking)
  if (success) {
    if (blockingMovement) {
      m_moveGroupPtr->move();
    } else {
      m_moveGroupPtr->asyncMove();
    }
  }

  return success;
}

bool ControlArm::sendToCmdPoses(std::vector<geometry_msgs::Pose> poses) {
  for (int i; i < poses.size(); ++i) {
    ROS_INFO_STREAM("[ControlArmNode] Visiting pose " << i);
    m_cmdPose.position = poses.at(i).position;
    m_cmdPose.orientation = poses.at(i).orientation;
    sendToCmdPose();
  }
  return true; 
}

bool ControlArm::sendToDeltaCmdPose() {

  geometry_msgs::Pose currPose = getCurrentEEPose(); 

  ROS_INFO_STREAM("[ControlArm] current EE Pose:" << currPose);
  geometry_msgs::Pose cmdPose;
  cmdPose.position.x = currPose.position.x + m_cmdDeltaPose.position.x;
  cmdPose.position.y = currPose.position.y + m_cmdDeltaPose.position.y;
  cmdPose.position.z = currPose.position.z + m_cmdDeltaPose.position.z;
  cmdPose.orientation.x = currPose.orientation.x + m_cmdDeltaPose.orientation.x;
  cmdPose.orientation.y = currPose.orientation.y + m_cmdDeltaPose.orientation.y;
  cmdPose.orientation.z = currPose.orientation.z + m_cmdDeltaPose.orientation.z;
  cmdPose.orientation.w = currPose.orientation.w + m_cmdDeltaPose.orientation.w;
  ROS_INFO_STREAM("[ControlArm] cmd EE Pose: " << cmdPose);

  // set CMD pose
  m_cmdPose = cmdPose;

  sendToCmdPose();

  return true;
}

geometry_msgs::Pose ControlArm::getCurrentEEPose() {
  
  // populate m_cmd pose --> TODO: Create separate method for this
  Eigen::Affine3d currPose = m_moveGroupPtr->getCurrentState()->getFrameTransform(EE_LINK_NAME); 
  geometry_msgs::Pose currROSPose;
  tf::poseEigenToMsg(currPose, currROSPose);

  return currROSPose;

}

void ControlArm::addCollisionObject(moveit_msgs::PlanningScene &planningScene) {

  ROS_INFO("Adding collision object...");
  std::vector<moveit_msgs::CollisionObject> collisionObjects;
  moveit_msgs::CollisionObject collisionObject1;
  moveit_msgs::CollisionObject collisionObject2;
  collisionObject1.header.frame_id = m_moveGroupPtr->getPlanningFrame();
  collisionObject2.header.frame_id = m_moveGroupPtr->getPlanningFrame();

  // Add table in basement
  collisionObject1.id = "table";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.5;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 0.02;

  // A table pose (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = -0.5;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.5;

  collisionObject1.primitives.push_back(primitive);
  collisionObject1.primitive_poses.push_back(table_pose);
  collisionObject1.operation = collisionObject1.ADD;

  collisionObjects.push_back(collisionObject1);

  for (std::size_t i = 0; i < collisionObjects.size(); ++i) {
    planningScene.world.collision_objects.push_back(collisionObjects.at(i));
  };

  ROS_INFO("Added collisions");
}

bool ControlArm::disableCollisionSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) 
{
  // TODO: Move this to specific script because it's related to magnetic
  // localization -> or grasping
  if (planSceneInit) {
    collision_detection::AllowedCollisionMatrix acm =
        m_planningScenePtr->getAllowedCollisionMatrix();

    // Before setting collisions
    acm.setEntry("powerline_cable1", "separator_right_head", true);

    moveit_msgs::PlanningScene planningScene;
    m_planningScenePtr->getPlanningSceneMsg(planningScene);
    // Create new collision matrix
    acm.getMessage(planningScene.allowed_collision_matrix);
    planningScene.is_diff = true;
    // m_planningScenePtr->setPlanningSceneMsg(planningScene); --> Setting it
    // over mPlanningScenePtr;

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planningScene;
    applyPlanningSceneSrvCli.call(srv);

    bool debugOut = false;
    if (debugOut) {
      acm.print(std::cout);
      ROS_INFO("[ControlArm] Disabled collisions: %d",
               (bool)srv.response.success);
      collision_detection::AllowedCollisionMatrix acm_after =
          m_planningScenePtr->getAllowedCollisionMatrix();
      acm_after.print(std::cout);
    }
    return true;
  } else {
    return false;
  }
}

void ControlArm::getRunningControllers(std::vector<std::string> &runningControllerNames) {
  ROS_INFO("[ControlArm] Listing controllers: ");
  controller_manager_msgs::ListControllersRequest listReq;
  controller_manager_msgs::ListControllersResponse listRes;
  listCtlSrvCli.call(listReq, listRes);
  // ROS_INFO_STREAM("[ControlArm] Controllers: " << listRes);

  for (std::size_t i = 0; i < listRes.controller.size(); ++i) {
    if (listRes.controller[i].state == "running") {
      // Additional constraints for controllers that must be active all of the
      // time
      // TODO: Add excluded controller list
      if (listRes.controller[i].name != "joint_state_controller" &&
          listRes.controller[i].name != "distancer_right_position_controller" &&
          listRes.controller[i].name != "distancer_left_position_controller") {
        runningControllerNames.push_back(listRes.controller[i].name);

        ROS_INFO_STREAM(
            "[ControlArm] Stopping controller: " << listRes.controller[i].name);
      }
    }
  }

  // ROS_INFO_STREAM("[ControlArm] Running controllers are: " <<
  // runningControllerNames);
}

bool ControlArm::startPositionCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Starting JointPosition controllers...");
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  // Stop running controllers
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_1_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_2_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_3_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_4_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_5_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_6_position_controller"));
  switchControllerRequest.start_asap = true;
  // Controller Manager: To switch controllers you need to specify a strictness
  // level of controller_manager_msgs::SwitchController::STRICT (2) or
  // ::BEST_EFFORT (1). Defaulting to ::BEST_EFFORT.
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchCtlSrvCli.call(switchControllerRequest, switchControllerResponse);
  ros::Duration(0.1).sleep();

  // sendZeros("position");

  return switchControllerResponse.ok;
}

bool ControlArm::startJointTrajCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Starting JointTrajectoryController...");
  
  // Stop running controllers
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  // NOTE: "arm_controller" is hardcoded!
  switchControllerRequest.start_controllers.push_back(std::string("arm_controller"));
  switchControllerRequest.start_asap = true;
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchCtlSrvCli.call(switchControllerRequest, switchControllerResponse);

  return switchControllerResponse.ok;
}

bool ControlArm::startJointGroupPositionCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Starting JointGroupPositionController...");
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  // Stop running controllers
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  switchControllerRequest.start_controllers.push_back(std::string("joint_group_position_controller"));
  switchControllerRequest.start_asap = true;
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchCtlSrvCli.call(switchControllerRequest, switchControllerResponse);
  ros::Duration(0.5).sleep();
  // TODO: Add enabling stuff for different controller type
  ROS_INFO("Sending all joints to zero");

  ros::Duration(0.5).sleep();
  // TODO: Add method for sending current joint states

  return switchControllerResponse.ok;
}

bool ControlArm::startJointGroupVelocityCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Start JointGroupVelocityController...");
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  // Stop running controllers
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_group_velocity_controller"));
  switchControllerRequest.start_asap = true;
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchCtlSrvCli.call(switchControllerRequest, switchControllerResponse);

  ROS_INFO("Switched to velocity controller");

  return switchControllerResponse.ok;
}

bool ControlArm::addCollisionObjectSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  ROS_INFO("Entered collision object");
  // Initialize planning scene
  moveit_msgs::PlanningScene planningScene;
  m_planningScenePtr->getPlanningSceneMsg(planningScene);
  ROS_INFO("Got planning scene.");
  addCollisionObject(planningScene);
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planningScene;
  applyPlanningSceneSrvCli.call(srv);
  return true;
  // How to add this to planning scene moveit?
  // http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
}

//bool ControlArm::getAnalyticIK(const geometry_msgs::Pose wantedPose)
//{
// TODO: Add analytic IK for Franka if exists
//
//}

Eigen::MatrixXd ControlArm::getJacobian(Eigen::Vector3d refPointPosition) {

  Eigen::MatrixXd jacobianMatrix;
  m_currentRobotStatePtr->getJacobian(
      m_jointModelGroupPtr,
      m_currentRobotStatePtr->getLinkModel(
          m_jointModelGroupPtr->getLinkModelNames().back()),
      refPointPosition, jacobianMatrix);

  return jacobianMatrix;
}

std::vector<geometry_msgs::Pose> ControlArm::createCartesianWaypoints(geometry_msgs::Pose startPose, geometry_msgs::Pose endPose, int numPoints) {
    std::vector<geometry_msgs::Pose> result;
    geometry_msgs::Pose pose_;
    if (numPoints <= 1) {
        result.push_back(startPose);
        return result;
    }
    double stepX = (endPose.position.x - startPose.position.x) / (numPoints - 1);
    double stepY = (endPose.position.y - startPose.position.y) / (numPoints - 1); 
    double stepZ = (endPose.position.z - startPose.position.z) / (numPoints - 1); 
    // Set i == 1 because start point doesn't have to be included into waypoint list 
    // https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/
    for (int i = 1; i < numPoints; ++i) {
        pose_.position.x = startPose.position.x + i * stepX; 
        pose_.position.y = startPose.position.y + i * stepY; 
        pose_.position.z = startPose.position.z + i * stepZ; 
        pose_.orientation.x = startPose.orientation.x; 
        pose_.orientation.y = startPose.orientation.y;
        pose_.orientation.z = startPose.orientation.z;
        pose_.orientation.w = startPose.orientation.w; 
        result.push_back(pose_);
    }

    // Print the values
    std::cout << "Cartesian Waypoints values: ";
    for (const auto& val : result) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    return result;
}

float ControlArm::round(float var) {

  float value = (int)(var * 1000 + .5);
  return (float)value / 1000;
}

void ControlArm::run() {

  ros::Rate r(25);
  
  while (ros::ok) {
    // Get current joint position for every joint in robot arm
    getArmState();

    // Get all joints
    m_jointModelGroupPtr = m_currentRobotStatePtr->getJointModelGroup(GROUP_NAME);
    Eigen::Affine3d currentPose_ = m_moveGroupPtr->getCurrentState()->getFrameTransform(EE_LINK_NAME);
    geometry_msgs::Pose currentROSPose_; tf::poseEigenToMsg(currentPose_, currentROSPose_);
    currPosePub.publish(currentROSPose_);

    if (robotState == IDLE)
    {
      ROS_INFO_STREAM_THROTTLE(5, "uam_ros_ctl is in IDLE mode.");
    }
    else{
      ROS_INFO_STREAM_THROTTLE(5, "uam_ros_ctl is in "<< stateNames[robotState] << "mode.");
    }

    if (robotState != SERVO_CTL && servoEntered) {servoPtr->setPaused(true); servoEntered=false;} 

    if(robotState == JOINT_TRAJ_CTL) 
    {
      if (recivPoseCmd)
        sendToCmdPose(); 
    }

    if (robotState == SERVO_CTL)
    {   
      if (!servoEntered)
      {   
        // Moveit servo status codes: https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/include/moveit_servo/utils/datatypes.hpp  
        servoPtr->start(); 
        servoEntered = true; 
        ROS_INFO_STREAM("[ControlArm] Turning servo on!");
        //servoPtr->getStatus(); 
            
      }
      ROS_INFO_STREAM("[ControlArm] servo loop on!");        
      }
      recivPoseCmd = false; 
    };

    ROS_INFO_STREAM_THROTTLE(60, "Current arm state is: " << stateNames[robotState]); 
    
    // Sleep
    r.sleep();

};



// TOOD: Move to utils
