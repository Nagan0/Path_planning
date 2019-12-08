import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

import rospy
import rospkg
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker

import sys
import copy
#from time import sleep
import time


class CalculateComplexityOffset():

    def __init__(self):
        rospy.init_node('calculate_complexity_offset', anonymous=True)
        self.pose_names = ["pos_x", "pos_y", "pos_z"]
        rospy.loginfo("Calculate Complexity Offset Node.")
    
    def load_trajectory_data(self):
        file = open("/home/shuntaro/denso_ws/src/denso_pkgs/denso_path_planning/documents/path.csv", 'r')
        lines = file.readlines()
        length = len(lines)
        
        pose_elements = {}

        line = []
        pos_x = []
        pos_y = []
        pos_z = []

        for i in range(0, length/6):
            pos_x.append(float(lines[6*i]))
            pos_y.append(float(lines[6*i+1]))
            pos_z.append(float(lines[6*i+2]))
            
        pose_elements[self.pose_names[0]] = np.array(pos_x)
        pose_elements[self.pose_names[1]] = np.array(pos_y)
        pose_elements[self.pose_names[2]] = np.array(pos_z)
        
        return pose_elements



    def calc_offset(self, pose_elements):
        #Linear interpolate based trajectory
        pose_elements_lin = {}
        length_lin = len(pose_elements["pos_x"])

        for i in range(0, 3):
            pose_elements_lin[self.pose_names[i]] = np.linspace(pose_elements[self.pose_names[i]][0], pose_elements[self.pose_names[i]][-1], length_lin)

        #Calc offset
        pose_offset = {}
        offset_list = []
        pos_offset_x = []
        pos_offset_y = []
        pos_offset_z = []
        total_distance = 0

        for i in range(0, 3):
            pose_offset[self.pose_names[i]] = pose_elements[self.pose_names[i]] - pose_elements_lin[self.pose_names[i]]
        
        for j in range(0, length_lin):
            offset_list.append(np.array([pose_offset["pos_x"][j], pose_offset["pos_y"][j], pose_offset["pos_z"][j]]))
            distance = np.linalg.norm(offset_list[j])
            print("Node[" + str(j) + "] distance: " + str(distance))
            total_distance += distance
  
        print("Total distance: " + str(total_distance))
        print("Number of node: " + str(length_lin))



    def callback(self):
        traj = self.load_trajectory_data()
        self.calc_offset(traj)


def main():
  
    CCO = CalculateComplexityOffset()
    CCO.callback()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")

if __name__ == '__main__':
    main()
