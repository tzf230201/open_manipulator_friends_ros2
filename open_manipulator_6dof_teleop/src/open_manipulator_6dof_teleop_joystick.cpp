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

#include "open_manipulator_6dof_teleop/open_manipulator_6dof_teleop_joystick.h"

#include <chrono>
#include <functional>

using namespace open_manipulator_teleop;
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace
{
template<typename ServiceT>
bool sendPlanningRequest(
  const rclcpp::Node::SharedPtr & node,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request,
  const std::string & service_name)
{
  if (!client->service_is_ready() && !client->wait_for_service(500ms))
  {
    RCLCPP_WARN(node->get_logger(), "Service '%s' is not available", service_name.c_str());
    return false;
  }

  client->async_send_request(
    request,
    [logger = node->get_logger(), service_name](typename rclcpp::Client<ServiceT>::SharedFuture future)
    {
      try
      {
        if (!future.get()->is_planned)
        {
          RCLCPP_WARN(logger, "Service '%s' returned is_planned=false", service_name.c_str());
        }
      }
      catch (const std::exception & exception)
      {
        RCLCPP_WARN(
          logger, "Service '%s' response failed: %s", service_name.c_str(), exception.what());
      }
    });

  return true;
}
}  // namespace

OM_TELEOP::OM_TELEOP(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  present_joint_angle.resize(NUM_OF_JOINT);
  present_kinematic_position.resize(3);
  end_effector_name_ = node_->declare_parameter<std::string>("end_effector_name", "gripper");

  initClient();
  initSubscriber();

  RCLCPP_INFO(node_->get_logger(), "OpenManipulator initialization");
}

OM_TELEOP::~OM_TELEOP()
{
}

void OM_TELEOP::initClient()
{
  goal_task_space_path_from_present_position_only_client_ =
    node_->create_client<SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_joint_space_path_client_ =
    node_->create_client<SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ =
    node_->create_client<SetJointPosition>("goal_tool_control");

}
void OM_TELEOP::initSubscriber()
{
  chain_joint_states_sub_ =
    node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&OM_TELEOP::jointStatesCallback, this, _1));
  chain_kinematics_pose_sub_ =
    node_->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "kinematics_pose", 10, std::bind(&OM_TELEOP::kinematicsPoseCallback, this, _1));
  joy_command_sub_ =
    node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&OM_TELEOP::joyCallback, this, _1));
}

void OM_TELEOP::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint5"))  temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint6"))  temp_angle.at(5) = (msg->position.at(i));
  }
  present_joint_angle = temp_angle;

}

void OM_TELEOP::kinematicsPoseCallback(
  const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position = temp_position;
}
void OM_TELEOP::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if(msg->axes.at(1) >= 0.9) setGoal("x+");
  else if(msg->axes.at(1) <= -0.9) setGoal("x-");
  else if(msg->axes.at(0) >=  0.9) setGoal("y+");
  else if(msg->axes.at(0) <= -0.9) setGoal("y-");
  else if(msg->buttons.at(3) == 1) setGoal("z+");
  else if(msg->buttons.at(0) == 1) setGoal("z-");
  else if(msg->buttons.at(5) == 1) setGoal("home");
  else if(msg->buttons.at(4) == 1) setGoal("init");

  if(msg->buttons.at(2) == 1) setGoal("gripper close");
  else if(msg->buttons.at(1) == 1) setGoal("gripper open");
}

std::vector<double> OM_TELEOP::getPresentJointAngle()
{
  return present_joint_angle;
}
std::vector<double> OM_TELEOP::getPresentKinematicsPose()
{
  return present_kinematic_position;
}

bool OM_TELEOP::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  return sendPlanningRequest<SetJointPosition>(
    node_, goal_joint_space_path_client_, request, "goal_joint_space_path");
}

bool OM_TELEOP::setToolControl(std::vector<double> joint_angle)
{
  auto request = std::make_shared<SetJointPosition::Request>();
  request->joint_position.joint_name.push_back(end_effector_name_);
  request->joint_position.position = joint_angle;

  return sendPlanningRequest<SetJointPosition>(
    node_, goal_tool_control_client_, request, "goal_tool_control");
}

bool OM_TELEOP::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  auto request = std::make_shared<SetKinematicsPose::Request>();
  request->planning_group = end_effector_name_;
  request->kinematics_pose.pose.position.x = kinematics_pose.at(0);
  request->kinematics_pose.pose.position.y = kinematics_pose.at(1);
  request->kinematics_pose.pose.position.z = kinematics_pose.at(2);
  request->path_time = path_time;

  return sendPlanningRequest<SetKinematicsPose>(
    node_, goal_task_space_path_from_present_position_only_client_, request,
    "goal_task_space_path_from_present_position_only");
}

void OM_TELEOP::setGoal(const std::string & command)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(6, 0.0);

  if(command == "x+")
  {
    printf("increase(++) x axis in cartesian space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(command == "x-")
  {
    printf("decrease(--) x axis in cartesian space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(command == "y+")
  {
    printf("increase(++) y axis in cartesian space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(command == "y-")
  {
    printf("decrease(--) y axis in cartesian space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(command == "z+")
  {
    printf("increase(++) z axis in cartesian space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(command == "z-")
  {
    printf("decrease(--) z axis in cartesian space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }

  else if(command == "gripper open")
  {
    printf("open gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if(command == "gripper close")
  {
    printf("close gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }

  else if(command == "home")
  {
    printf("home pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.78);
    joint_name.push_back("joint3"); joint_angle.push_back(1.5);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.8);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(command == "init")
  {
    printf("init pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.0);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("open_manipulator_6dof_teleop_joystick");
  OM_TELEOP om_teleop(node);

  RCLCPP_INFO(node->get_logger(), "OpenManipulator teleoperation using joystick start");
  rclcpp::spin(node);

  printf("Teleop. is finished\n");
  rclcpp::shutdown();
  return 0;
}
