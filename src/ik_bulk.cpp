#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


void callback(const geometry_msgs::PoseArray::ConstPtr& pose_array_msg)
{    
    std::cout << pose_array_msg->poses[0].position.x << std::endl;

    //geometry_msg::Pose target_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;

    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    moveit::planning_interface::MoveGroupInterface move_group("arm");

    const int num_of_points = pose_array_msg->poses.size();
    for(int j=0; j<num_of_points; j++)
    {
        waypoints.push_back(pose_array_msg->poses[j]);
    }

    move_group.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //ROS_INFO_NAMED("Visualizing plan", fraction * 100.0);

    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    for(std::size_t i=0; i<waypoints.size(); i++)
    {
	visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
    }
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_bulk");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //moveit::planning_interface::MoveGroupInterface group("arm");
   

    ros::Subscriber transformed_traj_sub = nh.subscribe<geometry_msgs::PoseArray>("transformed_traj", 10, callback);
    

/*
    ros::Subscriber transformed_traj_sub {nh.subscribe<geometry_msgs::PoseArray>("transformed_traj", 1,
        std::function<void (const geometry_msgs::PoseArray::ConstPtr&)>
	{
            [&](const auto& target)
            {	    
	        std::cout << "=====Subscribed pose array=====" << std::endl;
                group.setStartState(*group.getCurrentState());

                const unsigned int num_of_points = target->poses.size();

		robot_state::RobotState robot_state_goal(*group.getCurrentState());
		const robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

		moveit_msgs::TrajectoryConstraints initial_traj;
		std::vector<moveit_msgs::Constraints> all_joint_constraints;

                std::string joint_names[6] = {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};

		std::map<std::string, double> joints;
		std::map<std::string, double> init_joints;
                bool ik;
		bool is_first_point = true;

		for(int j = 0; j < num_of_points; j++)
		{
		    std::cout << "===Now processing waypoint" << j << "===" << std::endl;
                    std::vector<moveit_msgs::JointConstraint> joint_con_at_t;
		    joint_model_group = group.getCurrentState()->getJointModelGroup("arm");

		    std::cout << "[Position]" << std::endl;
		    std::cout << "x: " << target->poses[j].position.x << std::endl;
                    std::cout << "y: " << target->poses[j].position.y << std::endl;
                    std::cout << "z: " << target->poses[j].position.z << std::endl;
                    std::cout << "[Orientation]" << std::endl;
		    std::cout << "w: " << target->poses[j].orientation.w << std::endl;
		    std::cout << "x: " << target->poses[j].orientation.x << std::endl;
		    std::cout << "y: " << target->poses[j].orientation.y << std::endl;
		    std::cout << "z: " << target->poses[j].orientation.z << std::endl;


                    ik = robot_state_goal.setFromIK(joint_model_group, target->poses[j], 10, 1);
                    if(!ik)
		    {
		        ROS_ERROR_STREAM("Inverse kinematics could not obtain a valid joint angles.");
		    }
		    else
		    {
			for(int k = 0; k < 6; k++)
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


		


	    }
	}

    )};
*/

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
