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
    for(int j=(num_of_points-1); j>=0; j--)
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
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_bulk");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber transformed_traj_sub = nh.subscribe<geometry_msgs::PoseArray>("transformed_traj", 10, callback);
    


    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
