#include <iostream>
#include <map>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("arm");

  group.setStartState(*group.getCurrentState());

  robot_state::RobotState robot_state_start(*group.getCurrentState());

  geometry_msgs::Pose target;
  target.orientation.w = -0.000152221;
  target.orientation.x = 1.0;
  target.orientation.y = -9.82654e-07;
  target.orientation.z = -0.000537402;
  target.position.x = 0.28726;
  // target.position.x = 28726;
  target.position.y = 4.20769e-06;
  target.position.z = 0.604027;

  group.setPoseTarget(target);

  robot_state::RobotStatePtr robot_state_goal = group.getCurrentState();
  const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

  bool ik = robot_state_goal->setFromIK(joint_model_group, target, 4, 1);
  std::cout << "bool ik = " << ik << std::endl;

  std::map<std::string, double> joints;
  joints["joint_1"] = robot_state_goal->getVariablePosition("joint_1");
  joints["joint_2"] = robot_state_goal->getVariablePosition("joint_2");
  joints["joint_3"] = robot_state_goal->getVariablePosition("joint_3");
  joints["joint_4"] = robot_state_goal->getVariablePosition("joint_4");
  joints["joint_5"] = robot_state_goal->getVariablePosition("joint_5");
  joints["joint_6"] = robot_state_goal->getVariablePosition("joint_6");
  group.setJointValueTarget(joints);

  std::cout << "====Joint Angle====" << std::endl;
  std::cout << "joint_1: " << joints["joint_1"] << std::endl;
  std::cout << "joint_2: " << joints["joint_2"] << std::endl;
  std::cout << "joint_3: " << joints["joint_3"] << std::endl;
  std::cout << "joint_4: " << joints["joint_4"] << std::endl;
  std::cout << "joint_5: " << joints["joint_5"] << std::endl;
  std::cout << "joint_6: " << joints["joint_6"] << std::endl;

  moveit::planning_interface::MoveGroup::Plan result_plan;
  group.plan(result_plan);
  group.move();

  ros::spinOnce();
  ros::shutdown();

}
