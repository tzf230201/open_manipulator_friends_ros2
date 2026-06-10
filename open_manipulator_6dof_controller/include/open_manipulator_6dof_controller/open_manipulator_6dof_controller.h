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

#ifndef OPEN_MANIPULATOR_6DOF_CONTROLLER_H
#define OPEN_MANIPULATOR_6DOF_CONTROLLER_H

#include <pthread.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "open_manipulator_msgs/msg/joint_position.hpp"
#include "open_manipulator_msgs/msg/kinematics_pose.hpp"
#include "open_manipulator_msgs/msg/open_manipulator_state.hpp"
#include "open_manipulator_msgs/srv/get_joint_position.hpp"
#include "open_manipulator_msgs/srv/get_kinematics_pose.hpp"
#include "open_manipulator_msgs/srv/set_actuator_state.hpp"
#include "open_manipulator_msgs/srv/set_drawing_trajectory.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#include "open_manipulator_6dof_libs/open_manipulator.h"

namespace open_manipulator_controller
{

class OpenManipulatorController
{
private:
  using JointPosition = open_manipulator_msgs::msg::JointPosition;
  using KinematicsPose = open_manipulator_msgs::msg::KinematicsPose;
  using OpenManipulatorState = open_manipulator_msgs::msg::OpenManipulatorState;
  using SetJointPosition = open_manipulator_msgs::srv::SetJointPosition;
  using SetKinematicsPose = open_manipulator_msgs::srv::SetKinematicsPose;
  using SetDrawingTrajectory = open_manipulator_msgs::srv::SetDrawingTrajectory;
  using SetActuatorState = open_manipulator_msgs::srv::SetActuatorState;
  using GetJointPosition = open_manipulator_msgs::srv::GetJointPosition;
  using GetKinematicsPose = open_manipulator_msgs::srv::GetKinematicsPose;

  rclcpp::Node::SharedPtr node_;

  bool using_platform_;
  bool using_moveit_;
  double control_period_;

  rclcpp::Publisher<OpenManipulatorState>::SharedPtr open_manipulator_states_pub_;
  std::vector<rclcpp::Publisher<KinematicsPose>::SharedPtr> open_manipulator_kinematics_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr open_manipulator_joint_states_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gazebo_goal_joint_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr moveit_update_start_state_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr open_manipulator_option_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_planned_path_sub_;

  rclcpp::Service<SetJointPosition>::SharedPtr goal_joint_space_path_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_pose_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_position_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_orientation_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_task_space_path_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_task_space_path_position_only_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_task_space_path_orientation_only_server_;
  rclcpp::Service<SetJointPosition>::SharedPtr goal_joint_space_path_from_present_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_orientation_only_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_server_;
  rclcpp::Service<SetJointPosition>::SharedPtr goal_tool_control_server_;
  rclcpp::Service<SetActuatorState>::SharedPtr set_actuator_state_server_;
  rclcpp::Service<SetDrawingTrajectory>::SharedPtr goal_drawing_trajectory_server_;
  rclcpp::Service<GetJointPosition>::SharedPtr get_joint_position_server_;
  rclcpp::Service<GetKinematicsPose>::SharedPtr get_kinematics_pose_server_;
  rclcpp::Service<SetJointPosition>::SharedPtr set_joint_position_server_;
  rclcpp::Service<SetKinematicsPose>::SharedPtr set_kinematics_pose_server_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  trajectory_msgs::msg::JointTrajectory joint_trajectory_;
  double moveit_sampling_time_;
  bool moveit_plan_only_;

  pthread_t timer_thread_;

  OpenManipulator open_manipulator_;

  bool tool_ctrl_state_;
  bool timer_thread_state_;
  bool moveit_plan_state_;

public:
  OpenManipulatorController(const rclcpp::Node::SharedPtr & node, std::string usb_port, std::string baud_rate);
  ~OpenManipulatorController();

  void publishCallback();

  void initPublisher();
  void initSubscriber();
  void initServer();

  void openManipulatorOptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void displayPlannedPathCallback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg);

  double getControlPeriod(void) { return control_period_; }

  bool goalJointSpacePathCallback(SetJointPosition::Request & req, SetJointPosition::Response & res);
  bool goalJointSpacePathToKinematicsPoseCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalJointSpacePathToKinematicsPositionCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalJointSpacePathToKinematicsOrientationCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalTaskSpacePathCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalTaskSpacePathPositionOnlyCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalTaskSpacePathOrientationOnlyCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalJointSpacePathFromPresentCallback(SetJointPosition::Request & req, SetJointPosition::Response & res);
  bool goalTaskSpacePathFromPresentCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalTaskSpacePathFromPresentPositionOnlyCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalTaskSpacePathFromPresentOrientationOnlyCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool goalToolControlCallback(SetJointPosition::Request & req, SetJointPosition::Response & res);
  bool setActuatorStateCallback(SetActuatorState::Request & req, SetActuatorState::Response & res);
  bool goalDrawingTrajectoryCallback(SetDrawingTrajectory::Request & req, SetDrawingTrajectory::Response & res);
  bool setJointPositionMsgCallback(SetJointPosition::Request & req, SetJointPosition::Response & res);
  bool setKinematicsPoseMsgCallback(SetKinematicsPose::Request & req, SetKinematicsPose::Response & res);
  bool getJointPositionMsgCallback(GetJointPosition::Request & req, GetJointPosition::Response & res);
  bool getKinematicsPoseMsgCallback(GetKinematicsPose::Request & req, GetKinematicsPose::Response & res);

  void startTimerThread();
  static void * timerThread(void * param);

  void moveitTimer(double present_time);
  void process(double time);

  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishJointStates();
  void publishGazeboCommand();

  bool calcPlannedPath(const std::string planning_group, JointPosition msg);
  bool calcPlannedPath(const std::string planning_group, KinematicsPose msg);
};

}  // namespace open_manipulator_controller

#endif  // OPEN_MANIPULATOR_6DOF_CONTROLLER_H
