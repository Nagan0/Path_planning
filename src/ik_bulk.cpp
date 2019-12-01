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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_solver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("arm");

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

		std::cout << target->poses[1].position.x << std::endl;


	    }
	}

    )};

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
