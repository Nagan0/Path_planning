#!/usr/bin/env python
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy
import rospkg
import tf
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker


class InitializeMovingTrajectory:

    def __init__(self):
        rospy.init_node('initialize_moving_trajectory', anonymous=False)
        self.tf_listener = tf.TransformListener()
        self.pointcloud_sub = rospy.Subscriber("human_side/moving", PoseArray, self.callback, queue_size=1, buff_size=2**24)
        self.trajectory_pub = rospy.Publisher("human_traj", PoseArray, queue_size = 1)
        self.num_of_waypoints_after_upsampling = 20
        self.interpolate_method = 'quadratic'
        # self.offset_x = 0.2
        # self.offset_z = 0.4
        self.offset_x = 0.0
        self.offset_z = 0.3
        self.wrist_offset = rospy.get_param('offset', 0.10)
        self.grasping_wrist_point_frame = rospy.get_param('grasping_wrist_point_frame', 'part_0_wrist_point_0')
        self.assembling_wrist_point_frame = rospy.get_param('assembling_wrist_point_frame', 'assembling_target_part_wrist_point_0')
        self.source_frame = rospy.get_param('source_frame', 'world')
        self.pose_names = ["ori_w", "ori_x", "ori_y", "ori_z", "pos_x", "pos_y", "pos_z"]
        rospy.loginfo("Initialized InitializeMovingTrajectoryNode.")
        rospy.loginfo("num_of_waypoints_after_upsampling: "+str(self.num_of_waypoints_after_upsampling))
        rospy.loginfo("upsampling interpolate method: "+self.interpolate_method)
        rospy.loginfo("offset_x: "+str(self.offset_x))
        rospy.loginfo("offset_z: "+str(self.offset_z))

    def down_sampling(self, pose_array, down_sampling_rate, offset=False):
        poses = pose_array.poses
        array_length = len(poses)
        interval = array_length * down_sampling_rate
        index_array = np.linspace(0, array_length-1, interval, dtype ='int')
        # print index_array
        # print len(index_array)

        pose_elements = {}

        ori_w = []
        ori_x = []
        ori_y = []
        ori_z = []
        pos_x = []
        pos_y = []
        pos_z = []

        for i in index_array:
            # ori_w.append(poses[i].orientation.w)
            # ori_x.append(poses[i].orientation.x)
            # ori_y.append(poses[i].orientation.y)
            # ori_z.append(poses[i].orientation.z)
            ori_w.append(0.0)
            ori_x.append(1.0)
            ori_y.append(0.0)
            ori_z.append(0.0)
            if offset:
                pos_x.append(poses[i].position.x+self.offset_x)
                pos_y.append(poses[i].position.y)
                pos_z.append(poses[i].position.z+self.offset_z)
            else:
                pos_x.append(poses[i].position.x)
                pos_y.append(poses[i].position.y)
                pos_z.append(poses[i].position.z)

        pose_elements[self.pose_names[0]] = np.array(ori_w)
        pose_elements[self.pose_names[1]] = np.array(ori_x)
        pose_elements[self.pose_names[2]] = np.array(ori_y)
        pose_elements[self.pose_names[3]] = np.array(ori_z)
        pose_elements[self.pose_names[4]] = np.array(pos_x)
        pose_elements[self.pose_names[5]] = np.array(pos_y)
        pose_elements[self.pose_names[6]] = np.array(pos_z)

        return pose_elements

    def up_sampling(self, pose_elements):
        pose_elements_ = {}
        t_raw = np.linspace(0, 1, len(pose_elements[self.pose_names[0]]))
        plt.scatter(t_raw, pose_elements[self.pose_names[0]], c='red', s=80, marker='x',  label='raw')
        t = np.linspace(0, 1, self.num_of_waypoints_after_upsampling)
        for pose_name in self.pose_names:
            pose_elements_[pose_name] = interpolate.interp1d(t_raw, pose_elements[pose_name], kind=self.interpolate_method)(t)
        return pose_elements_

    def add_offset(self, pose_elements):
        # Linear interpolate between grasp point and asm point
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

        pose_measure ={}
        pose_measure_lin ={}
        ori_w = np.array([grasp_rot[3], asm_rot[3]])
        ori_x = np.array([grasp_rot[0], asm_rot[0]])
        ori_y = np.array([grasp_rot[1], asm_rot[1]])
        ori_z = np.array([grasp_rot[2], asm_rot[2]])
        pos_x = np.array([grasp_trans[0], asm_trans[0]])
        pos_y = np.array([grasp_trans[1], asm_trans[1]])
        pos_z = np.array([grasp_trans[2]+self.wrist_offset, asm_trans[2]+self.wrist_offset])

        pose_measure[self.pose_names[0]] = ori_w
        pose_measure[self.pose_names[1]] = ori_x
        pose_measure[self.pose_names[2]] = ori_y
        pose_measure[self.pose_names[3]] = ori_z
        pose_measure[self.pose_names[4]] = pos_x
        pose_measure[self.pose_names[5]] = pos_y
        pose_measure[self.pose_names[6]] = pos_z

        t_raw = np.linspace(0, 1, 2)
        t = np.linspace(0, 1, len(pose_elements[self.pose_names[0]]))
        for pose_name in self.pose_names:
            pose_measure_lin[pose_name] = interpolate.interp1d(t_raw, pose_measure[pose_name], kind='linear')(t)

        # Linear interpolate between human traj start point and goal point
        pose_human_lin = {}
        ori_w = np.array([pose_elements[self.pose_names[0]][0], pose_elements[self.pose_names[0]][-1]])
        ori_x = np.array([pose_elements[self.pose_names[1]][0], pose_elements[self.pose_names[1]][-1]])
        ori_y = np.array([pose_elements[self.pose_names[2]][0], pose_elements[self.pose_names[2]][-1]])
        ori_z = np.array([pose_elements[self.pose_names[3]][0], pose_elements[self.pose_names[3]][-1]])
        pos_x = np.array([pose_elements[self.pose_names[4]][0], pose_elements[self.pose_names[4]][-1]])
        pos_y = np.array([pose_elements[self.pose_names[5]][0], pose_elements[self.pose_names[5]][-1]])
        pos_z = np.array([pose_elements[self.pose_names[6]][0], pose_elements[self.pose_names[6]][-1]])

        pose_human_lin[self.pose_names[0]] = ori_w
        pose_human_lin[self.pose_names[1]] = ori_x
        pose_human_lin[self.pose_names[2]] = ori_y
        pose_human_lin[self.pose_names[3]] = ori_z
        pose_human_lin[self.pose_names[4]] = pos_x
        pose_human_lin[self.pose_names[5]] = pos_y
        pose_human_lin[self.pose_names[6]] = pos_z

        t_raw = np.linspace(0, 1, 2)
        t = np.linspace(0, 1, len(pose_elements[self.pose_names[0]]))
        for pose_name in self.pose_names:
            pose_human_lin[pose_name] = interpolate.interp1d(t_raw, pose_human_lin[pose_name], kind='linear')(t)

        # Calc offset
        pose_offset = {}
        pose_elements_ = {}
        for pose_name in self.pose_names:
            pose_offset[pose_name] = pose_elements[pose_name] - pose_human_lin[pose_name]

            # TODO: consider orientation
            if pose_name[0] == 'o':
                pose_elements_[pose_name] = pose_measure_lin[pose_name]
                continue

            pose_elements_[pose_name] = pose_measure_lin[pose_name] + pose_offset[pose_name]
        return pose_elements_

    def dict_to_posearray(self, pose_dict):
        pose_array_msg = PoseArray()
        for i in range(len(pose_dict["ori_w"])):
            pose_msg = Pose()
            pose_msg.orientation.w = pose_dict["ori_w"][i]
            pose_msg.orientation.x = pose_dict["ori_x"][i]
            pose_msg.orientation.y = pose_dict["ori_y"][i]
            pose_msg.orientation.z = pose_dict["ori_z"][i]
            pose_msg.position.x = pose_dict["pos_x"][i]
            pose_msg.position.y = pose_dict["pos_y"][i]
            pose_msg.position.z = pose_dict["pos_z"][i]
            pose_array_msg.poses.append(pose_msg)
        return pose_array_msg

    def publish_initial_trajectory(self, pose_elements_upsampled):
        # pose_elements_upsampled["pos_z"] += np.full_like(pose_elements_upsampled["pos_z"], 0.1)
        pose_array_msg = self.dict_to_posearray(pose_elements_upsampled)
        self.trajectory_pub.publish(pose_array_msg)

    def callback(self, data):
        pose_elements_downsampled = self.down_sampling(data, 0.4, offset=True)
        pose_elements_upsampled = self.up_sampling(pose_elements_downsampled)
        pose_elements_add_offset = self.add_offset(pose_elements_upsampled)
        pose_elements_downsampled2 = self.down_sampling(self.dict_to_posearray(pose_elements_add_offset), 0.7)
        rospy.loginfo("real data: "+str(len(data.poses)))
        rospy.loginfo("downsampled data: "+str(len(pose_elements_downsampled["pos_x"])))
        rospy.loginfo("upsampled data: "+str(len(pose_elements_upsampled["pos_x"])))
        rospy.loginfo("add offset data: "+str(len(pose_elements_add_offset["pos_x"])))
        plot_trajectory(pose_elements_downsampled, pose_elements_upsampled, pose_elements_add_offset, pose_elements_downsampled2)
        # print pose_elements_upsampled["pos_x"][0]
        # print pose_elements_upsampled["pos_y"][0]
        # print pose_elements_upsampled["pos_z"][0]
        self.publish_initial_trajectory(pose_elements_downsampled2)

def plot_trajectory(downsampled_elements, upsampled_elements, add_offset_elements, add_offset_downsampled_elements):

    fig = plt.figure(1)
    ax = fig.gca(projection='3d')

    downsampled_x = downsampled_elements["pos_x"]
    downsampled_y = downsampled_elements["pos_y"]
    downsampled_z = downsampled_elements["pos_z"]

    upsampled_x = upsampled_elements["pos_x"]
    upsampled_y = upsampled_elements["pos_y"]
    upsampled_z = upsampled_elements["pos_z"]

    add_offset_x = add_offset_elements["pos_x"]
    add_offset_y = add_offset_elements["pos_y"]
    add_offset_z = add_offset_elements["pos_z"]

    add_offset_downsampled_x = add_offset_downsampled_elements["pos_x"]
    add_offset_downsampled_y = add_offset_downsampled_elements["pos_y"]
    add_offset_downsampled_z = add_offset_downsampled_elements["pos_z"]

    ax.set_xlabel(r'$x$ [m]',fontsize=14)
    ax.set_ylabel(r'$y$ [m]',fontsize=14)
    ax.set_zlabel(r'$z$ [m]',fontsize=14)
    # ax.set_xlim(0, 0.5)
    # ax.set_ylim(-0.25, 0.25)
    ax.set_zlim(0, 0.5)
    ax.plot([0], [0], [0], 'o', color='y', ms=6 , label='world origin')
    # ax.plot(downsampled_x, downsampled_y, downsampled_z, "o", color="black", ms=6, mew=0.5, label='down-sampled data')
    # ax.plot(upsampled_x, upsampled_y, upsampled_z, "o", color="m", ms=3, mew=0.5, label='up-sampled data')
    ax.plot(add_offset_x, add_offset_y, add_offset_z, "o", color="g", ms=3, mew=0.5, label='add-offset data')
    ax.plot(add_offset_downsampled_x, add_offset_downsampled_y, add_offset_downsampled_z, "o", color="c", ms=6, label='down-sampled offset data')
    ax.plot([add_offset_x[0]], [add_offset_y[0]], [add_offset_z[0]], "o", color="b", ms=6, label='moving START')
    ax.plot([add_offset_x[-1]], [add_offset_y[-1]], [add_offset_z[-1]], "o", color="r", ms=6, label='moving GOAL')
    ax.legend()
    fig.suptitle('moving')
    plt.tight_layout()
    plt.show()


def main():
    InitializeMovingTrajectory()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")


if __name__ == '__main__':
    main()

