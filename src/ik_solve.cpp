#include <iostream>
#include <map>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_stomp_solver");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("arm");

  ros::Subscriber human_traj_sub {nh.subscribe<geometry_msgs::PoseArray>("human_traj", 1,
    std::function<void (const geometry_msgs::PoseArray::ConstPtr&)>
    {
      [&](const auto& target)
      {
        std::cout << "=====SUBSCRIBED POSE ARRAY=====" << std::endl;
        group.setStartState(*group.getCurrentState());

        const unsigned int num_of_points = target->poses.size();

        robot_state::RobotState robot_state_goal(*group.getCurrentState());
        const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

        moveit_msgs::TrajectoryConstraints initial_traj;
        std::vector<moveit_msgs::Constraints> all_joint_constraints;

        std::string joint_names[6] = {"joint_1","joint_2","joint_3","joint_4", "joint_5","joint_6"};

        std::map<std::string, double> joints;
        std::map<std::string, double> init_joints;
        bool ik;
        bool is_first_point = true;

        for(int j=0; j<num_of_points; j++)
        {
          std::cout << "Now processing waypoint " << j << std::endl;
          std::vector<moveit_msgs::JointConstraint> joint_con_at_t;
          joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

          std::cout << "===Target pose===" << std::endl;
          std::cout << "[Orientation]" << std::endl;
          std::cout << "w: " << target->poses[j].orientation.w << std::endl;
          std::cout << "x: " << target->poses[j].orientation.x << std::endl;
          std::cout << "y: " << target->poses[j].orientation.y << std::endl;
          std::cout << "z: " << target->poses[j].orientation.z << std::endl;
          std::cout << "[Position]" << std::endl;
          std::cout << "x: " << target->poses[j].position.x << std::endl;
          std::cout << "y: " << target->poses[j].position.y << std::endl;
          std::cout << "z: " << target->poses[j].position.z << std::endl;
          std::cout << "=================" << std::endl;

          ik = robot_state_goal.setFromIK(joint_model_group, target->poses[j], 4, 1);
          if(!ik)
          {
            ROS_ERROR_STREAM("Inverse kinematics could not obtain a valid joint angles.");
          }
          else
          {
            for(int k=0; k<6; k++)
            {
              moveit_msgs::JointConstraint joint_con;
              joint_con.joint_name = joint_names[k];
              joint_con.position = robot_state_goal.getVariablePosition(joint_names[k]);
              joints[joint_names[k]] = robot_state_goal.getVariablePosition(joint_names[k]);
              joint_con.weight = 1.0;
              joint_con_at_t.push_back(joint_con);
            }
            if(is_first_point)
            {
              is_first_point = false;
              init_joints = joints;
            }
            moveit_msgs::Constraints con;
            con.joint_constraints = joint_con_at_t;
            all_joint_constraints.push_back(con);
          }
        }

        robot_state::RobotState robot_state_init(*group.getCurrentState());
        group.setStartState(robot_state_init);
        group.setJointValueTarget(init_joints);
        moveit::planning_interface::MoveGroup::Plan init_plan;
        group.plan(init_plan);
        group.execute(init_plan);

        initial_traj.constraints = all_joint_constraints;
        robot_state::RobotState robot_state_start(*group.getCurrentState());
        group.setStartState(robot_state_start);
        group.setTrajectoryConstraints(initial_traj);
        group.setJointValueTarget(joints);

        moveit::planning_interface::MoveGroup::Plan result_plan;
        group.plan(result_plan);
        group.execute(result_plan);
      }
    }
  )};

  while(ros::ok())
  {
    ros::spinOnce();
  }

  return 0;

}
