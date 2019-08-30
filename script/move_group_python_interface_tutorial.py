#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal        A list of floats, a Pose or a PoseStamped
        @param: actual      A list of floats, a Pose of a PoseStamped
        @param: tolerance   A float
        @returns: bool
        """
        all_epual = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)


        robot = moveit_commander.RobotCommander()


        scene = moveit_commander.PlanningSceneInterface()


        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)


        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


        planning_frame = group.get_planning_frame()
        print "========== Reference frame: %s" % planning_frame


        eef_link = group.get_end_effector_link()
        print "========== End effector: %s" % eef_link


        group_names = robot.get_group_names()
        print "========== Robot Groups:", robot.get_group_names()


        print "========== Printing robot state"
        print robot.get_current_state()
        print ""


        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names



    def go_to_joint_state(self):
        group = self.group


        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        #joint_goal[6] = 0


        group.go(joint_goal, wait=True)


        group.stop()


        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self):
        group = self.group


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        group.set_pose_target(pose_goal)


        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()


        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):
        group = self.group


        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1
        wpose.position.y += scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))


        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)


        return plan, fraction


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher


        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory);


    def execute_plan(self, plan):
        group = self.group

        group.execute(plan, wait=True)


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached ==  is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False


    def add_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "gripper"
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = 'gripper'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def detach_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False,timeout=timeout)



def main():
    try:
        print "========== Press 'Enter' to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
        raw_input()
        tutorial = MoveGroupPythonIntefaceTutorial()

        print "========== Press 'Enter' to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_joint_state()

        print "========== Press 'Enter' to execute a movement using a pose goal ..."
        raw_input()
        tutorial.go_to_pose_goal()

        print "========== Press 'Enter' to plan and display a Cartesian path ..."
        raw_input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        print "========== Press 'Enter' to display a save trajectory (this will replay the Cartesian path) ..."
        raw_input()
        tutorial.display_trajectory(cartesian_plan)

        print "========== Press 'Enter' to execute a saved path ..."
        raw_input()
        tutorial.execute_plan(cartesian_plan)

        print "========== Press 'Enter' to add a box to the planning scene ..."
        raw_input()
        tutorial.add_box()

        print "========== Press 'Enter' to attach a Box to the vs087 ..."
        raw_input()
        tutorial.attach_box()

        print "========== Press 'Enter' to plan and execute a path with an attached collision object ..."
        raw_input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        tutorial.execute_plan(cartesian_plan)

        print "========== Press 'Enter' to detach the boxfrom the vs087 ..."
        raw_input()
        tutorial.detach_box()

        print "========== Press 'Enter' to remove the box from the planning scene ..."
        raw_input()
        tutorial.remove_box()

        print "========== Python tutorial demo complete!"

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return



if __name__ == '__main__':
    main()




