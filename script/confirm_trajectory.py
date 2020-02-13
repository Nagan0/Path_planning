#!/usr/bin/env python
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker

import sys
import copy
#from time import sleep
import time

from denso_path_planning_srvs.srv import planning
from denso_path_planning_srvs.srv import planningResponse
from denso_path_planning_srvs.srv import trajectory
from denso_path_planning_srvs.srv import trajectoryResponse



class TransformTrajectory():

    def __init__(self):
        rospy.init_node('transform_trajectory', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")

        self.trajectory_pub = rospy.Publisher("transformed_traj", PoseArray, queue_size = 1)
        self.target_srv = rospy.Service("denso_path_planning/transform_trajectory/moving", planning, self.callback)

        self.pose_names = ["pos_x", "pos_y", "pos_z", "ori_w", "ori_x", "ori_y", "ori_z"]
        rospy.loginfo("Transform Trajectory Node.")

    def get_trajectory_data(self, service_elements):
        traj_name = service_elements.path_name

        pose_elements = {}
        pos_x = []
        pos_y = []
        pos_z = []
        ori_w = []
        ori_x = []
        ori_y = []
        ori_z = []

        rospy.wait_for_service('denso_path_planning/base_trajectory')
        try:
            print("Now loading...")
            traj_query = rospy.ServiceProxy('denso_path_planning/base_trajectory', trajectory)
            traj_response = traj_query(traj_name)
            num_of_poses = len(traj_response.base_trajectory.poses)
            
            if num_of_poses == 0:
                print("Loading trajectory is failed...")
            else:
                print("Loading trajectory is success!!")

            for i in range(0, num_of_poses):
                pos_x.append(traj_response.base_trajectory.poses[i].position.x)
                pos_y.append(traj_response.base_trajectory.poses[i].position.y)
                pos_z.append(traj_response.base_trajectory.poses[i].position.z)
                ori_w.append(traj_response.base_trajectory.poses[i].orientation.w)
                ori_x.append(traj_response.base_trajectory.poses[i].orientation.x)
                ori_y.append(traj_response.base_trajectory.poses[i].orientation.y)
                ori_z.append(traj_response.base_trajectory.poses[i].orientation.z)

            pose_elements[self.pose_names[0]] = np.array(pos_x)
            pose_elements[self.pose_names[1]] = np.array(pos_y)
            pose_elements[self.pose_names[2]] = np.array(pos_z)
            pose_elements[self.pose_names[3]] = np.array(ori_w)
            pose_elements[self.pose_names[4]] = np.array(ori_x)
            pose_elements[self.pose_names[5]] = np.array(ori_y)
            pose_elements[self.pose_names[6]] = np.array(ori_z)

            print(str(pose_elements['pos_x'][0]))

            return pose_elements

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def calc_offset(self, pose_elements, service_elements):
        #Linear interpolate based trajectory
        pose_elements_lin = {}
        length_lin = len(pose_elements["pos_x"])

        for i in range(0, 3):
            pose_elements_lin[self.pose_names[i]] = np.linspace(pose_elements[self.pose_names[i]][0], pose_elements[self.pose_names[i]][-1], length_lin)


        pose_target_lin = {}

        target_start_x = 0.3022
        target_start_y = 0.2506
        target_start_z = 0.2811
        '''
        target_goal_x = 0.376011
        target_goal_y = -0.22364
        target_goal_z = 0.240476
        '''
        pose_target_lin[self.pose_names[0]] = np.linspace(target_start_x, service_elements.target_pose.pose.position.x, length_lin)
        pose_target_lin[self.pose_names[1]] = np.linspace(target_start_y, service_elements.target_pose.pose.position.y, length_lin)
        pose_target_lin[self.pose_names[2]] = np.linspace(target_start_z, service_elements.target_pose.pose.position.z, length_lin)


        #Calc offset
        pose_offset = {}
        pose_new_trajectory = {} 

        for i in range(0, 3):
            pose_offset[self.pose_names[i]] = pose_elements[self.pose_names[i]] - pose_elements_lin[self.pose_names[i]]
            pose_new_trajectory[self.pose_names[i]] = pose_target_lin[self.pose_names[i]] + pose_offset[self.pose_names[i]]


        #add rotation
        for i in range(3, 7):
            pose_new_trajectory[self.pose_names[i]] = pose_elements[self.pose_names[i]]
       
        '''
        pose_new_trajectory["ori_w"][0] = arm_current_pose.orientation.w
        pose_new_trajectory["ori_x"][0] = arm_current_pose.orientation.x
        pose_new_trajectory["ori_y"][0] = arm_current_pose.orientation.y
        pose_new_trajectory["ori_z"][0] = arm_current_pose.orientation.z
        pose_new_trajectory["ori_w"][-1] = service_elements.target_pose.pose.orientation.w
        pose_new_trajectory["ori_x"][-1] = service_elements.target_pose.pose.orientation.x
        pose_new_trajectory["ori_y"][-1] = service_elements.target_pose.pose.orientation.y
        pose_new_trajectory["ori_z"][-1] = service_elements.target_pose.pose.orientation.z
        '''

        return pose_new_trajectory



    def dict_to_posearray(self, pose_dict):
        pose_array = PoseArray()
        for i in range(len(pose_dict["pos_x"])):
            pose_msg = Pose()
            pose_msg.position.x = pose_dict["pos_x"][i]
            pose_msg.position.y = pose_dict["pos_y"][i]
            pose_msg.position.z = pose_dict["pos_z"][i]
            pose_msg.orientation.w = pose_dict["ori_w"][i]
            pose_msg.orientation.x = pose_dict["ori_x"][i]
            pose_msg.orientation.y = pose_dict["ori_y"][i]
            pose_msg.orientation.z = pose_dict["ori_z"][i]
            pose_array.poses.append(pose_msg)
        return pose_array



    def publish_transformed_trajectory(self, pose_pose):
        pose_array_msg = self.dict_to_posearray(pose_pose)
        self.trajectory_pub.publish(pose_array_msg)

        #test
        return trajectoryResponse(success)


    
    def plot_trajectory(self, pose_first, pose_second):
        fig = plt.figure(1)
        ax = fig.gca(projection = '3d')
      
        pos_first_x = pose_first["pos_x"]
        pos_first_y = pose_first["pos_y"]
        pos_first_z = pose_first["pos_z"]

        pos_second_x = pose_second["pos_x"]
        pos_second_y = pose_second["pos_y"]
        pos_second_z = pose_second["pos_z"]

        ax.set_xlabel(r'$x$ [m]', fontsize = 14)
        ax.set_ylabel(r'$y$ [m]', fontsize = 14)
        ax.set_zlabel(r'$z$ [m]', fontsize = 14)
        
        ax.set_ylim(-0.25, 0.25)
        
        ax.plot([0], [0], [0], 'o', color = 'y', ms = 6, label = 'world origin')

        ax.plot(pos_first_x, pos_first_y, pos_first_z, "o", color = "c", ms =6, label = 'trajectory data')
        ax.plot(pos_second_x, pos_second_y, pos_second_z, "o", color = "b", ms = 6, label = 'new trajectory data')
        ax.plot([pos_second_x[0]], [pos_second_y[0]], [pos_second_z[0]], "o", color = "r", ms = 6, label = 'moving START')
        ax.plot([pos_second_x[-1]], [pos_second_y[-1]], [pos_second_z[-1]], "o", color = "g", ms = 6, label = 'moving GOAL')

        ax.legend()
        fig.suptitle('moving')
        plt.tight_layout()
        plt.pause(0.01)
    
    


    def callback(self, data):
        #traj = self.load_trajectory_data()
        traj = self.get_trajectory_data(data)
        resp = planningResponse()
        resp.trajectory_name = data.path_name

        if not traj:
            print("Get trajectory failed: %s")
            resp.success = False 
            
        else:
            new_traj = self.calc_offset(traj, data)
            resp.success = True
        
        return resp
        #self.plot_trajectory(traj, new_traj)
        #self.publish_transformed_trajectory(new_traj)
        #self.plot_trajectory(traj, new_traj)



def main():
    TransformTrajectory()
    #t1 = time.time()

    #TT = TransformTrajectory()
    #TT.callback()

    #t2 = time.time()
    #t3 = t2 - t1
    #print("TIME:" + str(t3))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")

if __name__ == '__main__':
    main()
