#!/usr/bin/env python
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy
import rospkg
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker

import sys
import copy
from time import sleep


class BulkPickandPlace():

    def __init__(self):
        rospy.init_node('BulkPickandPlace', anonymous=True)
        #self.target_sub = rospy.Subscriber("bulk_trajectory", PoseArray, self.callback, queue_size=1,buff_size2**24)
        self.trajectory_pub = rospy.Publisher("transformed_traj", PoseArray, queue_size = 1)

        #self.down_sampling_rate = 0.2
        #self.num_of_waypoints_after_upsampling = 20
        #self.interpolate_method = 'quadratic'
        #self.offset_x = 0.2
        #self.offset_z = 0.4
        #self.offset_x = 0.0
        #self.offset_z = 0.3
        self.pose_names = ["pos_x", "pos_y", "pos_z", "rot_1", "rot_2", "rot_3"]
        rospy.loginfo("Bulk Pick and Place Node.")
        #rospy.loginfo("down_sampling_rate: "+str(self.down_sampling_rate))
        #rospy.loginfo("num_of_waypoints_after_upsampling: "+str(self.num_of_waypoints_after_upsampling))
        #rospy.loginfo("upsampling interpolate method: "+self.interpolate_method)
        #rospy.loginfo("offset_x: "+str(self.offset_x))
        #rospy.loginfo("offset_z: "+str(self.offset_z))


    
    def load_trajectory_data(self):
        file = open("/home/shuntaro/denso_ws/src/denso_pkgs/denso_path_planning/documents/path.csv", 'r')
        lines = file.readlines()
        length = len(lines)
        
        pose_elements = {}

        line = []
        pos_x = []
        pos_y = []
        pos_z = []
        rot_1 = []
        #rot_12 = []
        #rot_13 = []
        rot_2 = []
        #rot_22 = []
        #rot_23 = []
        rot_3 = []
        #rot_32 = []
        #rot_33 = []

        for i in range(0, length/6):
            pos_x.append(float(lines[6*i]))
            pos_y.append(float(lines[6*i + 1]))
            pos_z.append(float(lines[6*i + 2]))
            rot_1.append(lines[6*i + 3])
            #rot_12.append(float(line[4]))
            #rot_13.append(float(line[5]))
            rot_2.append(lines[6*i + 4])
            #rot_22.append(float(line[7]))
            #rot_23.append(float(line[8]))
            rot_3.append(lines[6*i + 5])
            #rot_32.append(float(line[10]))
            #rot_33.append(float(line[11]))
     
        pose_elements[self.pose_names[0]] = np.array(pos_x)
        pose_elements[self.pose_names[1]] = np.array(pos_y)
        pose_elements[self.pose_names[2]] = np.array(pos_z)
        pose_elements[self.pose_names[3]] = rot_1
        pose_elements[self.pose_names[4]] = rot_2
        pose_elements[self.pose_names[5]] = rot_3

        return pose_elements



    def calc_offset(self, pose_elements):
        #Linear interpolate based trajectory
        pose_elements_lin = {}
        length_lin = len(pose_elements["pos_x"])

        for i in range(0, 3):
            pose_elements_lin[self.pose_names[i]] = np.linspace(pose_elements[self.pose_names[i]][0], pose_elements[self.pose_names[i]][-1], length_lin)


        #Linear interpolate between grasp point and asm point
        """
        tf_time = rospy.Time(0)
        while not rospy.is_shutdown():
            try:
                grasp_trans, grasp_rot = self.tf_listener.lookupTransform(self.source_frame, self.grasping_wrist_point_frame, tf_time)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        while not rospy.is_shutdown():
            try:
                asm_trans, asm_rot = self.tf_listener.lookupTransform(self.source_frame, self.assembling_wrist_point_frame, tf_time)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        """

        pose_target_lin = {}

        target_start_x = 0.601
        target_start_y = 0.201
        target_start_z = 0.501

        target_goal_x = 0.288
        target_goal_y = 0.000
        target_goal_z = 0.604

        pose_target_lin[self.pose_names[0]] = np.linspace(target_start_x, target_goal_x, length_lin)
        pose_target_lin[self.pose_names[1]] = np.linspace(target_start_y, target_goal_y, length_lin)
        pose_target_lin[self.pose_names[2]] = np.linspace(target_start_z, target_goal_z, length_lin)


        #Calc offset
        pose_offset = {}
        pose_new_trajectory = {} 

        for i in range(0, 3):
            pose_offset[self.pose_names[i]] = pose_elements[self.pose_names[i]] - pose_elements_lin[self.pose_names[i]]
            pose_new_trajectory[self.pose_names[i]] = pose_target_lin[self.pose_names[i]] + pose_offset[self.pose_names[i]]


        #add rotation
        for i in range(3, 6):
            pose_new_trajectory[self.pose_names[i]] = pose_elements[self.pose_names[i]]


        #return pose_elements_lin
        #return pose_target_lin
        return pose_new_trajectory



    def dict_to_posearray(self, pose_dict):
        pose_array_msg = PoseArray()

        for i in range(len(pose_dict["pos_x"])):
            pose_msg = Pose()
            pose_msg.position.x = pose_dict["pos_x"][i]
            pose_msg.position.y = pose_dict["pos_y"][i]
            pose_msg.position.z = pose_dict["pos_z"][i]

        return pose_array_msg



    def publish_transformed_trajectory(self, pose_pose):
        pose_array_msg = self.dict_to_posearray(pose_pose)
        self.trajectory_pub.publish(pose_array_msg)



    def plot_trajectory(self, pose_elements, pose_new_trajectory):
        fig = plt.figure(1)
        ax = fig.gca(projection = '3d')

        pos_x = pose_elements["pos_x"]
        pos_y = pose_elements["pos_y"]
        pos_z = pose_elements["pos_z"]

        pos_new_x = pose_new_trajectory["pos_x"]
        pos_new_y = pose_new_trajectory["pos_y"]
        pos_new_z = pose_new_trajectory["pos_z"]

        ax.set_xlabel(r'$x$ [m]', fontsize = 14)
        ax.set_ylabel(r'$y$ [m]', fontsize = 14)
        ax.set_zlabel(r'$z$ [m]', fontsize = 14)
        #ax.set_xlim(0, 0.5)
        ax.set_ylim(-0.25, 0.25)
        #ax.set_zlim(0, 0.5)
        ax.plot([0], [0], [0], 'o', color = 'y', ms = 6, label = 'world origin')

        ax.plot(pos_x, pos_y, pos_z, "o", color = "c", ms =6, label = 'trajectory data')
        ax.plot(pos_new_x, pos_new_y, pos_new_z, "o", color = "b", ms = 6, label = 'new trajectory data')
        ax.plot([pos_new_x[0]], [pos_new_y[0]], [pos_new_z[0]], "o", color = "r", ms = 6, label = 'moving START')
        ax.plot([pos_new_x[-1]], [pos_new_y[-1]], [pos_new_z[-1]], "o", color = "g", ms = 6, label = 'moving GOAL')

        ax.legend()
        fig.suptitle('moving')
        plt.tight_layout()
        plt.show()



    #def callback(self, data):



def main():
    #BuldPickandPlace()

    BPP = BulkPickandPlace()
    
    traj = BPP.load_trajectory_data()
    linear = BPP.calc_offset(traj)
    BPP.plot_trajectory(traj, linear)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")

if __name__ == '__main__':
    main()
