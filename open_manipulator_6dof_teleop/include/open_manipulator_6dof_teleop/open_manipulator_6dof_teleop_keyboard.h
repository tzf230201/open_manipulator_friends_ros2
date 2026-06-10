#ifndef OPEN_MANIPULATOR_6DOF_TELEOP_KEYBOARD_H
#define OPEN_MANIPULATOR_6DOF_TELEOP_KEYBOARD_H

#include <termios.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "open_manipulator_msgs/msg/kinematics_pose.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"

#define NUM_OF_JOINT 6
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

namespace open_manipulator_teleop
{

class OM_TELEOP
{
private:
  using SetJointPosition = open_manipulator_msgs::srv::SetJointPosition;
  using SetKinematicsPose = open_manipulator_msgs::srv::SetKinematicsPose;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<SetJointPosition>::SharedPtr goal_joint_space_path_from_present_client_;
  rclcpp::Client<SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_client_;
  rclcpp::Client<SetJointPosition>::SharedPtr goal_joint_space_path_client_;
  rclcpp::Client<SetJointPosition>::SharedPtr goal_tool_control_client_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr chain_joint_states_sub_;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr chain_kinematics_pose_sub_;

  std::vector<double> present_joint_angle;
  std::vector<double> present_kinematic_position;

  struct termios oldt;
  std::string end_effector_name_;

public:
  explicit OM_TELEOP(const rclcpp::Node::SharedPtr & node);
  ~OM_TELEOP();

  void initClient();
  void initSubscriber();

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();

  bool setJointSpacePathFromPresent(
    std::vector<std::string> joint_name,
    std::vector<double> joint_angle,
    double path_time);
  bool setJointSpacePath(
    std::vector<std::string> joint_name,
    std::vector<double> joint_angle,
    double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(
    std::vector<double> kinematics_pose,
    double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  void printText();
  void setGoal(char ch);

  void restore_terminal_settings(void);
  void disable_waiting_for_enter(void);
};

}  // namespace open_manipulator_teleop

#endif  // OPEN_MANIPULATOR_6DOF_TELEOP_KEYBOARD_H
