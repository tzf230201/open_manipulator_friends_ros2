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

#include "open_manipulator_6dof_teleop/open_manipulator_6dof_teleop_keyboard.h"

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
  goal_joint_space_path_from_present_client_ =
    node_->create_client<SetJointPosition>("goal_joint_space_path_from_present");
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
}

void OM_TELEOP::jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))       temp_angle.at(0) = (msg->position.at(i));
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

std::vector<double> OM_TELEOP::getPresentJointAngle()
{
  return present_joint_angle;
}
std::vector<double> OM_TELEOP::getPresentKinematicsPose()
{
  return present_kinematic_position;
}

bool OM_TELEOP::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  return sendPlanningRequest<SetJointPosition>(
    node_, goal_joint_space_path_from_present_client_, request,
    "goal_joint_space_path_from_present");
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

void OM_TELEOP::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("---------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("y : increase joint 1 angle\n");
  printf("h : decrease joint 1 angle\n");
  printf("u : increase joint 2 angle\n");
  printf("j : decrease joint 2 angle\n");
  printf("i : increase joint 3 angle\n");
  printf("k : decrease joint 3 angle\n");
  printf("o : increase joint 4 angle\n");
  printf("l : decrease joint 4 angle\n");
  printf("p : increase joint 5 angle\n");
  printf("; : decrease joint 5 angle\n");
  printf("[ : increase joint 6 angle\n");
  printf("] : decrease joint 6 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");

  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf J5: %.3lf J6: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3),
         getPresentJointAngle().at(4),
         getPresentJointAngle().at(5));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");

}

void OM_TELEOP::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if(ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = -JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = JOINT_DELTA;
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = -JOINT_DELTA;
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = JOINT_DELTA;
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = -JOINT_DELTA;
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'p' || ch == 'P')
  {
    printf("input : p \tdecrease(++) joint 5 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5"); goalJoint.at(4) = JOINT_DELTA;
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == ';' || ch == ':')
  {
    printf("input : ; \tdecrease(--) joint 5 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5"); goalJoint.at(4) = -JOINT_DELTA;
    joint_name.push_back("joint6");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == '[' || ch == '{')
  {
    printf("input : [ \tdecrease(++) joint 6 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6"); goalJoint.at(5) = JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == ']' || ch == '}')
  {
    printf("input : ] \tdecrease(--) joint 6 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6"); goalJoint.at(5) = -JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }


  else if(ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if(ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }


  else if(ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.5;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.78);
    joint_name.push_back("joint3"); joint_angle.push_back(1.5);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.8);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.5;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.0);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

void OM_TELEOP::restore_terminal_settings(void)
{
  tcsetattr(0, TCSANOW, &oldt);  /* Apply saved settings */
}

void OM_TELEOP::disable_waiting_for_enter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt);  /* Save terminal settings */
  newt = oldt;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("open_manipulator_6dof_teleop_keyboard");
  OM_TELEOP om_teleop(node);

  RCLCPP_INFO(node->get_logger(), "OpenManipulator teleoperation using keyboard start");
  om_teleop.disable_waiting_for_enter();

  rclcpp::spin_some(node);
  om_teleop.printText();

  char ch;
  while (rclcpp::ok() && (ch = std::getchar()) != 'q')
  {
    rclcpp::spin_some(node);
    om_teleop.printText();
    rclcpp::spin_some(node);
    om_teleop.setGoal(ch);
  }

  printf("input : q \tTeleop. is finished\n");
  om_teleop.restore_terminal_settings();

  rclcpp::shutdown();
  return 0;
}
