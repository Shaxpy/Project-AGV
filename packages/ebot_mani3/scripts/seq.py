#!/usr/bin/env python2.7

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import roslib
import tf2_ros
from math import pi
from object_msgs.msg import ObjectPose

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point


# Creating Ur5Moveit class


class Ur5Moveit:

    # Constructor
    def __init__(self):
        global arm_group, hand_group

        self._planning_group = "arm_controller"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        arm_group = moveit_commander.MoveGroupCommander(self._planning_group)
        hand_group = moveit_commander.MoveGroupCommander("gripper_controller")
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        _planning_frame = arm_group.get_planning_frame()
        _eef_link = hand_group.get_end_effector_link()
        _group_names = self._robot.get_group_names()
        _box_name = ''

        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(_planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(_eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(_group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # A method to set the joint angles of the arm
    def set_joint_angles(self, arg_list_joint_angles):

        flag_plan = False

        while(flag_plan != True):

            list_joint_values = arm_group.get_current_joint_values()
            arm_group.set_joint_value_target(arg_list_joint_angles)
            arm_group.plan()
            flag_plan = arm_group.go(wait=True)

            list_joint_values = arm_group.get_current_joint_values()

            pose_values = arm_group.get_current_pose().pose

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed. Trying again" + '\033[0m')

        return flag_plan

    # A method to send the end effector to a given set of coordinates
    def go_to_pose(self, arg_pose):

        flag_plan = False

        while(flag_plan != True):

            pose_values = arm_group.get_current_pose().pose

            arm_group.set_pose_target(arg_pose)
            flag_plan = arm_group.go(wait=True)

            pose_values = arm_group.get_current_pose().pose

            list_joint_values = arm_group.get_current_joint_values()

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found. Trying again" + '\033[0m')

        return flag_plan

    # A method to set the gripping angle of the gripper
    def set_gripper_angles(self, arg_list_joint_angles):

        flag_plan = False

        while(flag_plan != True):

            list_joint_values = hand_group.get_current_joint_values()

            hand_group.set_joint_value_target(arg_list_joint_angles)
            hand_group.plan()
            flag_plan = hand_group.go(wait=True)

            list_joint_values = hand_group.get_current_joint_values()

            pose_values = hand_group.get_current_pose().pose

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed. Trying again" + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def client_coordinates(x, y, ori):

    # Initializing action client "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# Waiting for action server and listening for goals.
    client.wait_for_server()

# New goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    print ("going", x, y)
    # Move forward along the x axis of the "map" coordinate frame
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = ori[0]
    goal.target_pose.pose.orientation.w = ori[1]

    # Sending the goal to the action server.
    client.send_goal(goal)
    # Wait for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't exist, assume no Server is available
    if not wait:
        rospy.logerr("No Action server !")
        rospy.signal_shutdown("No Action server !")
    else:
        # Result of executing the action
        return client.get_result()


# Python node executed as main process
if __name__ == '__main__':
    try:
        rospy.init_node('taskfive', anonymous=True)

        ur5 = Ur5Moveit()

        north = [0.0, 1.0]
        south = [1.0, 0.0]
        east = [-0.7, 0.7]
        west = [0.7, 0.7]
        northeast = [-0.289, 0.957]
        southwest = [0.949, 0.315]

        # Arm pose for better movement
        movearm = [math.radians(-22),
                   math.radians(-25),
                   math.radians(20),
                   math.radians(-25),
                   math.radians(-29),
                   math.radians(-17)]

        pantry = [math.radians(172),
                  math.radians(-43),
                  math.radians(-84),
                  math.radians(228),
                  math.radians(-254),
                  math.radians(169)]

        ur5.set_joint_angles(movearm)

        # # pantry outside
        client_coordinates(12.96, 1.27, east)

        # # pantry inside
        client_coordinates(13.12, -1.0, north)

        # # pantry left table
        client_coordinates(14.4, -1.0, north)

        ur5.set_joint_angles(pantry)

        # rospy.sleep(10)

        # pantry left table turn toward south
        # client_coordinates(14.4, -1.0, south)

        # # pantry right table
        # client_coordinates(11.4, -1.0, south)

        # rospy.sleep(10)

        # # # pantry right table turn toward north
        # client_coordinates(11.6, -1.0, north)

        # # # # pantry inside
        # client_coordinates(13.12, -1.0, west)

        # # # # pantry outside
        # client_coordinates(12.96, 1.27, south)

        # # # # meeting outside
        # client_coordinates(8.57, 1.09, west)

        # # # # # meeting inside
        # client_coordinates(8.58, 2.71, south)

        # # # # # meeting drop box
        # client_coordinates(7.4, 2.71, north)

        # # # # meeting inside
        # client_coordinates(8.58, 2.71, east)

        # # # # meeting outside
        # client_coordinates(8.57, 1.09, north)

        # # research
        # client_coordinates(10.6, 9.48, east)

        # # store
        # client_coordinates(25.8, -3.05, northeast)

        # # store return
        # client_coordinates(25.8, -3.05, southwest)

        # # last room outside
        # client_coordinates(5.19, 0.85, east)

        # # last room inside v1
        # client_coordinates(5.6, -0.67, east)

        # # last room inside v2
        # client_coordinates(4.93, -1.18, east)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
