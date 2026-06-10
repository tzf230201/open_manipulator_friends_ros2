/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

/***********************************************************
** Modified by Hae-Bum Jung
************************************************************/

#include "open_manipulator_6dof_controller/open_manipulator_6dof_controller.h"

#include <chrono>
#include <cstdlib>
#include <functional>

using namespace open_manipulator_controller;
using std::placeholders::_1;

namespace
{
template<typename ServiceT, typename CallbackT>
typename rclcpp::Service<ServiceT>::SharedPtr createPrivateService(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  CallbackT callback)
{
  return node->create_service<ServiceT>(
    "~/" + name,
    [callback](
      const typename ServiceT::Request::SharedPtr request,
      typename ServiceT::Response::SharedPtr response)
    {
      callback(*request, *response);
    });
}
}  // namespace

OpenManipulatorController::OpenManipulatorController(
  const rclcpp::Node::SharedPtr & node,
  std::string usb_port,
  std::string baud_rate)
: node_(node),
  using_platform_(false),
  using_moveit_(false),
  control_period_(0.010f),
  moveit_sampling_time_(0.050f),
  moveit_plan_only_(true),
  tool_ctrl_state_(false),
  timer_thread_state_(false),
  moveit_plan_state_(false)
{
  control_period_ = node_->declare_parameter<double>("control_period", 0.010f);
  moveit_sampling_time_ = node_->declare_parameter<double>("moveit_sample_duration", 0.050f);
  using_platform_ = node_->declare_parameter<bool>("using_platform", false);
  using_moveit_ = node_->declare_parameter<bool>("using_moveit", false);
  moveit_plan_only_ = node_->declare_parameter<bool>("moveit_plan_only", true);
  const std::string planning_group_name =
    node_->declare_parameter<std::string>("planning_group_name", "arm");

  open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_);

  if (using_platform_)
  {
    log::info("Succeeded to init " + std::string(node_->get_fully_qualified_name()));
  }
  else
  {
    log::info(
      "Ready to simulate " + std::string(node_->get_fully_qualified_name()) + " on Gazebo");
  }

  if (using_moveit_)
  {
    move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_name);
    log::info("Ready to control " + planning_group_name + " group");
  }
}

OpenManipulatorController::~OpenManipulatorController()
{
  if (timer_thread_state_)
  {
    timer_thread_state_ = false;
    pthread_join(timer_thread_, nullptr);
  }

  log::info("Shutdown the OpenManipulator");
  open_manipulator_.disableAllActuator();
}

void OpenManipulatorController::startTimerThread()
{
  int error;
  if ((error = pthread_create(&this->timer_thread_, nullptr, this->timerThread, this)) != 0)
  {
    log::error("Creating timer thread failed!!", static_cast<double>(error));
    std::exit(EXIT_FAILURE);
  }
  timer_thread_state_ = true;
}

void * OpenManipulatorController::timerThread(void * param)
{
  OpenManipulatorController * controller = static_cast<OpenManipulatorController *>(param);
  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while (controller->timer_thread_state_)
  {
    next_time.tv_sec +=
      (next_time.tv_nsec + (static_cast<int>(controller->getControlPeriod() * 1000)) * 1000000) /
      1000000000;
    next_time.tv_nsec =
      (next_time.tv_nsec + (static_cast<int>(controller->getControlPeriod() * 1000)) * 1000000) %
      1000000000;

    const double time = next_time.tv_sec + (next_time.tv_nsec * 0.000000001);
    controller->process(time);

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    const double delta_nsec =
      controller->getControlPeriod() -
      ((next_time.tv_sec - curr_time.tv_sec) +
      (static_cast<double>(next_time.tv_nsec - curr_time.tv_nsec) * 0.000000001));

    if (delta_nsec > controller->getControlPeriod())
    {
      log::warn("Over the control time : ", delta_nsec);
      next_time = curr_time;
    }
    else
    {
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    }
  }
  return nullptr;
}

void OpenManipulatorController::initPublisher()
{
  const auto opm_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  for (const auto & name : opm_tools_name)
  {
    open_manipulator_kinematics_pose_pub_.push_back(
      node_->create_publisher<KinematicsPose>("~/" + name + "/kinematics_pose", 10));
  }

  open_manipulator_states_pub_ =
    node_->create_publisher<OpenManipulatorState>("~/states", 10);

  if (using_platform_)
  {
    open_manipulator_joint_states_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  }
  else
  {
    auto gazebo_joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
    gazebo_joints_name.reserve(gazebo_joints_name.size() + opm_tools_name.size());
    gazebo_joints_name.insert(gazebo_joints_name.end(), opm_tools_name.begin(), opm_tools_name.end());

    for (const auto & name : gazebo_joints_name)
    {
      gazebo_goal_joint_position_pub_.push_back(
        node_->create_publisher<std_msgs::msg::Float64>("~/" + name + "_position/command", 10));
    }
  }

  if (using_moveit_)
  {
    moveit_update_start_state_ =
      node_->create_publisher<std_msgs::msg::Empty>("/rviz/moveit/update_start_state", 10);
  }
}

void OpenManipulatorController::initSubscriber()
{
  open_manipulator_option_sub_ =
    node_->create_subscription<std_msgs::msg::String>(
    "~/option", 10, std::bind(&OpenManipulatorController::openManipulatorOptionCallback, this, _1));

  if (using_moveit_)
  {
    display_planned_path_sub_ =
      node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "/move_group/display_planned_path", 100,
      std::bind(&OpenManipulatorController::displayPlannedPathCallback, this, _1));
  }
}

void OpenManipulatorController::initServer()
{
  goal_joint_space_path_server_ =
    createPrivateService<SetJointPosition>(
    node_, "goal_joint_space_path",
    [this](auto & req, auto & res) { goalJointSpacePathCallback(req, res); });

  goal_joint_space_path_to_kinematics_pose_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_joint_space_path_to_kinematics_pose",
    [this](auto & req, auto & res) { goalJointSpacePathToKinematicsPoseCallback(req, res); });

  goal_joint_space_path_to_kinematics_position_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_joint_space_path_to_kinematics_position",
    [this](auto & req, auto & res) { goalJointSpacePathToKinematicsPositionCallback(req, res); });

  goal_joint_space_path_to_kinematics_orientation_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_joint_space_path_to_kinematics_orientation",
    [this](auto & req, auto & res) { goalJointSpacePathToKinematicsOrientationCallback(req, res); });

  goal_task_space_path_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_task_space_path",
    [this](auto & req, auto & res) { goalTaskSpacePathCallback(req, res); });

  goal_task_space_path_position_only_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_task_space_path_position_only",
    [this](auto & req, auto & res) { goalTaskSpacePathPositionOnlyCallback(req, res); });

  goal_task_space_path_orientation_only_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_task_space_path_orientation_only",
    [this](auto & req, auto & res) { goalTaskSpacePathOrientationOnlyCallback(req, res); });

  goal_joint_space_path_from_present_server_ =
    createPrivateService<SetJointPosition>(
    node_, "goal_joint_space_path_from_present",
    [this](auto & req, auto & res) { goalJointSpacePathFromPresentCallback(req, res); });

  goal_task_space_path_from_present_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_task_space_path_from_present",
    [this](auto & req, auto & res) { goalTaskSpacePathFromPresentCallback(req, res); });

  goal_task_space_path_from_present_position_only_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_task_space_path_from_present_position_only",
    [this](auto & req, auto & res) { goalTaskSpacePathFromPresentPositionOnlyCallback(req, res); });

  goal_task_space_path_from_present_orientation_only_server_ =
    createPrivateService<SetKinematicsPose>(
    node_, "goal_task_space_path_from_present_orientation_only",
    [this](auto & req, auto & res) { goalTaskSpacePathFromPresentOrientationOnlyCallback(req, res); });

  goal_tool_control_server_ =
    createPrivateService<SetJointPosition>(
    node_, "goal_tool_control",
    [this](auto & req, auto & res) { goalToolControlCallback(req, res); });

  set_actuator_state_server_ =
    createPrivateService<SetActuatorState>(
    node_, "set_actuator_state",
    [this](auto & req, auto & res) { setActuatorStateCallback(req, res); });

  goal_drawing_trajectory_server_ =
    createPrivateService<SetDrawingTrajectory>(
    node_, "goal_drawing_trajectory",
    [this](auto & req, auto & res) { goalDrawingTrajectoryCallback(req, res); });

  if (using_moveit_)
  {
    get_joint_position_server_ =
      createPrivateService<GetJointPosition>(
      node_, "moveit/get_joint_position",
      [this](auto & req, auto & res) { getJointPositionMsgCallback(req, res); });

    get_kinematics_pose_server_ =
      createPrivateService<GetKinematicsPose>(
      node_, "moveit/get_kinematics_pose",
      [this](auto & req, auto & res) { getKinematicsPoseMsgCallback(req, res); });

    set_joint_position_server_ =
      createPrivateService<SetJointPosition>(
      node_, "moveit/set_joint_position",
      [this](auto & req, auto & res) { setJointPositionMsgCallback(req, res); });

    set_kinematics_pose_server_ =
      createPrivateService<SetKinematicsPose>(
      node_, "moveit/set_kinematics_pose",
      [this](auto & req, auto & res) { setKinematicsPoseMsgCallback(req, res); });
  }
}

void OpenManipulatorController::openManipulatorOptionCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "print_open_manipulator_setting")
  {
    open_manipulator_.printManipulatorSetting();
  }

  if (msg->data == "switching_kinematics")
  {
    open_manipulator_.switchingKinematics();
  }
}

void OpenManipulatorController::displayPlannedPathCallback(
  const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
{
  if (msg->trajectory.empty())
  {
    log::warn("Received empty MoveIt planned trajectory");
    return;
  }

  joint_trajectory_ = msg->trajectory[0].joint_trajectory;

  if (!moveit_plan_only_)
  {
    log::println("[INFO] [OpenManipulator Controller] Execute MoveIt planned path", "GREEN");
    moveit_plan_state_ = true;
  }
  else
  {
    log::println("[INFO] [OpenManipulator Controller] Get MoveIt planned path", "GREEN");
  }
}

bool OpenManipulatorController::goalJointSpacePathCallback(
  SetJointPosition::Request & req,
  SetJointPosition::Response & res)
{
  std::vector<double> target_angle;

  for (std::size_t i = 0; i < req.joint_position.joint_name.size(); i++)
  {
    target_angle.push_back(req.joint_position.position.at(i));
  }

  open_manipulator_.makeJointTrajectory(target_angle, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsPoseCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(
    req.kinematics_pose.pose.orientation.w,
    req.kinematics_pose.pose.orientation.x,
    req.kinematics_pose.pose.orientation.y,
    req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeJointTrajectory(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsPositionCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.makeJointTrajectory(req.end_effector_name, target_pose.position, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsOrientationCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;

  Eigen::Quaterniond q(
    req.kinematics_pose.pose.orientation.w,
    req.kinematics_pose.pose.orientation.x,
    req.kinematics_pose.pose.orientation.y,
    req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeJointTrajectory(req.end_effector_name, target_pose.orientation, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(
    req.kinematics_pose.pose.orientation.w,
    req.kinematics_pose.pose.orientation.x,
    req.kinematics_pose.pose.orientation.y,
    req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);
  open_manipulator_.makeTaskTrajectory(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathPositionOnlyCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.makeTaskTrajectory(req.end_effector_name, position, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathOrientationOnlyCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  Eigen::Quaterniond q(
    req.kinematics_pose.pose.orientation.w,
    req.kinematics_pose.pose.orientation.x,
    req.kinematics_pose.pose.orientation.y,
    req.kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectory(req.end_effector_name, orientation, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathFromPresentCallback(
  SetJointPosition::Request & req,
  SetJointPosition::Response & res)
{
  std::vector<double> target_angle;

  for (std::size_t i = 0; i < req.joint_position.joint_name.size(); i++)
  {
    target_angle.push_back(req.joint_position.position.at(i));
  }

  open_manipulator_.makeJointTrajectoryFromPresentPosition(target_angle, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(
    req.kinematics_pose.pose.orientation.w,
    req.kinematics_pose.pose.orientation.x,
    req.kinematics_pose.pose.orientation.y,
    req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectoryFromPresentPose(
    req.planning_group, target_pose, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentPositionOnlyCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.makeTaskTrajectoryFromPresentPose(req.planning_group, position, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentOrientationOnlyCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  Eigen::Quaterniond q(
    req.kinematics_pose.pose.orientation.w,
    req.kinematics_pose.pose.orientation.x,
    req.kinematics_pose.pose.orientation.y,
    req.kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectoryFromPresentPose(
    req.planning_group, orientation, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalToolControlCallback(
  SetJointPosition::Request & req,
  SetJointPosition::Response & res)
{
  for (std::size_t i = 0; i < req.joint_position.joint_name.size(); i++)
  {
    open_manipulator_.makeToolTrajectory(
      req.joint_position.joint_name.at(i), req.joint_position.position.at(i));
  }

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::setActuatorStateCallback(
  SetActuatorState::Request & req,
  SetActuatorState::Response & res)
{
  if (req.set_actuator_state)
  {
    log::println("Wait a second for actuator enable", "GREEN");
    timer_thread_state_ = false;
    pthread_join(timer_thread_, nullptr);
    open_manipulator_.enableAllActuator();
    startTimerThread();
  }
  else
  {
    log::println("Wait a second for actuator disable", "GREEN");
    timer_thread_state_ = false;
    pthread_join(timer_thread_, nullptr);
    open_manipulator_.disableAllActuator();
    startTimerThread();
  }

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalDrawingTrajectoryCallback(
  SetDrawingTrajectory::Request & req,
  SetDrawingTrajectory::Response & res)
{
  try
  {
    if (req.drawing_trajectory_name == "circle")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];
      draw_circle_arg[1] = req.param[1];
      draw_circle_arg[2] = req.param[2];
      void * p_draw_circle_arg = &draw_circle_arg;

      open_manipulator_.makeCustomTrajectory(
        CUSTOM_TRAJECTORY_CIRCLE, req.end_effector_name, p_draw_circle_arg, req.path_time);
    }
    else if (req.drawing_trajectory_name == "line")
    {
      TaskWaypoint draw_line_arg;
      draw_line_arg.kinematic.position(0) = req.param[0];
      draw_line_arg.kinematic.position(1) = req.param[1];
      draw_line_arg.kinematic.position(2) = req.param[2];
      void * p_draw_line_arg = &draw_line_arg;

      open_manipulator_.makeCustomTrajectory(
        CUSTOM_TRAJECTORY_LINE, req.end_effector_name, p_draw_line_arg, req.path_time);
    }
    else if (req.drawing_trajectory_name == "rhombus")
    {
      double draw_rhombus_arg[3];
      draw_rhombus_arg[0] = req.param[0];
      draw_rhombus_arg[1] = req.param[1];
      draw_rhombus_arg[2] = req.param[2];
      void * p_draw_rhombus_arg = &draw_rhombus_arg;

      open_manipulator_.makeCustomTrajectory(
        CUSTOM_TRAJECTORY_RHOMBUS, req.end_effector_name, p_draw_rhombus_arg, req.path_time);
    }
    else if (req.drawing_trajectory_name == "heart")
    {
      double draw_heart_arg[3];
      draw_heart_arg[0] = req.param[0];
      draw_heart_arg[1] = req.param[1];
      draw_heart_arg[2] = req.param[2];
      void * p_draw_heart_arg = &draw_heart_arg;

      open_manipulator_.makeCustomTrajectory(
        CUSTOM_TRAJECTORY_HEART, req.end_effector_name, p_draw_heart_arg, req.path_time);
    }

    res.is_planned = true;
    return true;
  }
  catch (const std::exception &)
  {
    log::error("Creation the custom trajectory is failed!");
  }
  return true;
}

bool OpenManipulatorController::getJointPositionMsgCallback(
  GetJointPosition::Request & req,
  GetJointPosition::Response & res)
{
  (void) req;

  const std::vector<std::string> & joint_names = move_group_->getJointNames();
  std::vector<double> joint_values = move_group_->getCurrentJointValues();

  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    res.joint_position.joint_name.push_back(joint_names[i]);
    res.joint_position.position.push_back(joint_values[i]);
  }

  return true;
}

bool OpenManipulatorController::getKinematicsPoseMsgCallback(
  GetKinematicsPose::Request & req,
  GetKinematicsPose::Response & res)
{
  (void) req;

  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

  res.header = current_pose.header;
  res.kinematics_pose.pose = current_pose.pose;

  return true;
}

bool OpenManipulatorController::setJointPositionMsgCallback(
  SetJointPosition::Request & req,
  SetJointPosition::Response & res)
{
  JointPosition msg = req.joint_position;
  res.is_planned = calcPlannedPath(req.planning_group, msg);

  return true;
}

bool OpenManipulatorController::setKinematicsPoseMsgCallback(
  SetKinematicsPose::Request & req,
  SetKinematicsPose::Response & res)
{
  KinematicsPose msg = req.kinematics_pose;
  res.is_planned = calcPlannedPath(req.planning_group, msg);

  return true;
}

bool OpenManipulatorController::calcPlannedPath(
  const std::string planning_group,
  KinematicsPose msg)
{
  bool is_planned = false;
  geometry_msgs::msg::Pose target_pose = msg.pose;

  move_group_->setPoseTarget(target_pose);

  move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);
  move_group_->setGoalTolerance(msg.tolerance);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (!open_manipulator_.getMovingState())
  {
    const bool success =
      (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      is_planned = true;
    }
    else
    {
      log::warn("Failed to Plan (task space goal)");
      is_planned = false;
    }
  }
  else
  {
    log::warn("Robot is Moving");
    is_planned = false;
  }

  return is_planned;
}

bool OpenManipulatorController::calcPlannedPath(
  const std::string planning_group,
  JointPosition msg)
{
  bool is_planned = false;

  const robot_state::JointModelGroup * joint_model_group =
    move_group_->getCurrentState()->getJointModelGroup(planning_group);

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (std::size_t index = 0; index < msg.position.size(); index++)
  {
    joint_group_positions[index] = msg.position[index];
  }

  move_group_->setJointValueTarget(joint_group_positions);

  move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (!open_manipulator_.getMovingState())
  {
    const bool success =
      (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      is_planned = true;
    }
    else
    {
      log::warn("Failed to Plan (joint space goal)");
      is_planned = false;
    }
  }
  else
  {
    log::warn("Robot is moving");
    is_planned = false;
  }

  return is_planned;
}

void OpenManipulatorController::publishOpenManipulatorStates()
{
  OpenManipulatorState msg;
  if (open_manipulator_.getMovingState())
  {
    msg.open_manipulator_moving_state = msg.IS_MOVING;
  }
  else
  {
    msg.open_manipulator_moving_state = msg.STOPPED;
  }

  if (open_manipulator_.getActuatorEnabledState(JOINT_DYNAMIXEL))
  {
    msg.open_manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  }
  else
  {
    msg.open_manipulator_actuator_state = msg.ACTUATOR_DISABLED;
  }

  open_manipulator_states_pub_->publish(msg);
}

void OpenManipulatorController::publishKinematicsPose()
{
  KinematicsPose msg;
  const auto opm_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  uint8_t index = 0;
  for (const auto & tools : opm_tools_name)
  {
    KinematicPose pose = open_manipulator_.getKinematicPose(tools);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    open_manipulator_kinematics_pose_pub_.at(index)->publish(msg);
    index++;
  }
}

void OpenManipulatorController::publishJointStates()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();

  const auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  const auto tool_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  const auto joint_value = open_manipulator_.getAllActiveJointValue();
  const auto tool_value = open_manipulator_.getAllToolValue();

  for (std::size_t i = 0; i < joints_name.size(); i++)
  {
    msg.name.push_back(joints_name.at(i));

    msg.position.push_back(joint_value.at(i).position);
    msg.velocity.push_back(joint_value.at(i).velocity);
    msg.effort.push_back(joint_value.at(i).effort);
  }

  for (std::size_t i = 0; i < tool_name.size(); i++)
  {
    msg.name.push_back(tool_name.at(i));

    msg.position.push_back(tool_value.at(i).position);
    msg.velocity.push_back(0.0f);
    msg.effort.push_back(0.0f);
  }

  open_manipulator_joint_states_pub_->publish(msg);
}

void OpenManipulatorController::publishGazeboCommand()
{
  JointWaypoint joint_value = open_manipulator_.getAllActiveJointValue();
  JointWaypoint tool_value = open_manipulator_.getAllToolValue();

  for (std::size_t i = 0; i < joint_value.size(); i++)
  {
    std_msgs::msg::Float64 msg;
    msg.data = joint_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(i)->publish(msg);
  }

  for (std::size_t i = 0; i < tool_value.size(); i++)
  {
    std_msgs::msg::Float64 msg;
    msg.data = tool_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(joint_value.size() + i)->publish(msg);
  }
}

void OpenManipulatorController::publishCallback()
{
  if (using_platform_)
  {
    publishJointStates();
  }
  else
  {
    publishGazeboCommand();
  }

  publishOpenManipulatorStates();
  publishKinematicsPose();
}

void OpenManipulatorController::moveitTimer(double present_time)
{
  static double priv_time = 0.0f;
  static uint32_t step_cnt = 0;

  if (moveit_plan_state_)
  {
    const double path_time = present_time - priv_time;
    if (path_time > moveit_sampling_time_)
    {
      JointWaypoint target;
      const uint32_t all_time_steps = joint_trajectory_.points.size();

      for (std::size_t i = 0; i < joint_trajectory_.points[step_cnt].positions.size(); i++)
      {
        JointValue temp;
        temp.position = joint_trajectory_.points[step_cnt].positions.at(i);
        temp.velocity = joint_trajectory_.points[step_cnt].velocities.at(i);
        temp.acceleration = joint_trajectory_.points[step_cnt].accelerations.at(i);
        target.push_back(temp);
      }
      open_manipulator_.makeJointTrajectory(target, path_time);

      step_cnt++;
      priv_time = present_time;

      if (step_cnt >= all_time_steps)
      {
        step_cnt = 0;
        moveit_plan_state_ = false;
        if (moveit_update_start_state_->get_subscription_count() == 0)
        {
          log::warn("Could not update the start state! Enable External Communications at the MoveIt plugin");
        }
        std_msgs::msg::Empty msg;
        moveit_update_start_state_->publish(msg);
      }
    }
  }
  else
  {
    priv_time = present_time;
  }
}

void OpenManipulatorController::process(double time)
{
  moveitTimer(time);
  open_manipulator_.processOpenManipulator(time);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("open_manipulator_6dof_controller");

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (non_ros_args.size() < 3)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Please set '-port_name' and '-baud_rate' arguments for connected Dynamixels");
    rclcpp::shutdown();
    return 0;
  }

  usb_port = non_ros_args[1];
  baud_rate = non_ros_args[2];

  OpenManipulatorController om_controller(node, usb_port, baud_rate);

  om_controller.initPublisher();
  om_controller.initSubscriber();
  om_controller.initServer();
  om_controller.startTimerThread();

  auto publish_timer = node->create_wall_timer(
    std::chrono::duration<double>(om_controller.getControlPeriod()),
    [&om_controller]() { om_controller.publishCallback(); });

  (void) publish_timer;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
