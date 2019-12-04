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

void callback(const geometry_msgs::PoseArray::ConstPtr& pose_array_msg)
{
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    
    //move_group.setStartState(*move_group.getCurrentState());
    //robot_state::RobotState robot_state_goal(*move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup("arm");

    moveit_msgs::TrajectoryConstraints initial_traj;
    std::vector<moveit_msgs::Constraints> all_joint_constraints;

    std::string joint_names[6] = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
    
    //std::map<std::string, double> joints;

    std::cout << pose_array_msg->poses[0] << std::endl;
    
//---IK TEST---
    geometry_msgs::Pose target;
    target.orientation.w = -0.00022685;
    target.position.x = 0.63565447;
    target.position.y = 0.135733727;
    target.position.z = 0.54390942;
    target.orientation.x = -0.99999987;
    target.orientation.y = -0.00008291;
    target.orientation.z = -0.00045939;

    move_group.setPoseTarget(target);
    robot_state::RobotStatePtr robot_state_goal = move_group.getCurrentState();
    //const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup("arm");
    bool ik = robot_state_goal->setFromIK(joint_model_group, target, 4, 1);
    std::cout << "bool ik = " << ik << std::endl;

    std::map<std::string, double> joints;
    joints["joint_1"] = robot_state_goal->getVariablePosition("joint_1");
    joints["joint_2"] = robot_state_goal->getVariablePosition("joint_2");
    joints["joint_3"] = robot_state_goal->getVariablePosition("joint_3");
    joints["joint_4"] = robot_state_goal->getVariablePosition("joint_4");
    joints["joint_5"] = robot_state_goal->getVariablePosition("joint_5");
    joints["joint_6"] = robot_state_goal->getVariablePosition("joint_6");
    move_group.setJointValueTarget(joints);

    std::cout << "===Joint Angle===" << std::endl;
    std::cout << "joint_1: "<< joints["joint_1"] << std::endl;
    std::cout << "joint_2: "<< joints["joint_2"] << std::endl;
    std::cout << "joint_3: "<< joints["joint_3"] << std::endl;
    std::cout << "joint_4: "<< joints["joint_4"] << std::endl;
    std::cout << "joint_5: "<< joints["joint_5"] << std::endl;
    std::cout << "joint_6: "<< joints["joint_6"] << std::endl;

    moveit::planning_interface::MoveGroup::Plan result_plan;
    move_group.plan(result_plan);
    move_group.move();


//------



/*
    //for(int j=0; j<num_of_points; j++)
    std::vector<moveit_msgs::JointConstraint> joint_con_at_t;
    bool found_ik = robot_state_goal.setFromIK(joint_model_group, pose_array_msg->poses[0], 4,1);
    if (found_ik)
    {
        for(int k=0; k<6; k++)
	{
	    moveit_msgs::JointConstraint joint_con;
	    joint_con.joint_name = joint_names[k];
	    joint_con.position = robot_state_goal.getVariablePosition(joint_names[k]);
	    joints[joint_names[k]] = robot_state_goal.getVariablePosition(joint_names[k]);
	    joint_con.weight = 1.0;
	    joint_con_at_t.push_back(joint_con);

	    std::cout << joints[joint_names[1]] << std::endl;
	}
    }else
    {
        ROS_ERROR_STREAM("Inverse kinematics could not obtain a valid joint angles.");
    }
   
*/    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_stomp_solver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //moveit::planning_interface::MoveGroupInterface group("arm");
    ros::Subscriber transformed_traj_sub = nh.subscribe<geometry_msgs::PoseArray>("transformed_traj", 1, callback);
    //callback();

    while(ros::ok())
    {
        ros::spinOnce();
    }

/*    
    ros::spinOnce();
    ros::shutdown();
*/

    return 0;

}
