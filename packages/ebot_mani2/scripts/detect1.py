#! /usr/bin/env python

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
from geometry_msgs.msg import Point

global battery, coke, glue
coke = Point()
battery = Point()
glue = Point()


def listener_coord():

    flag = False
    while(flag != True):
        rospy.Subscriber('/detection_info', ObjectPose, callback)
        if(glue.z != 0.0):
            flag = True


def callback(data):

    # rospy.loginfo("Data Received : (%s , %.2f, %.2f , %.2f)", data.name, data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z )
    if(data.name == "Battery" and data.pose.header.seq == 1):

        battery.x = data.pose.pose.position.x
        battery.y = data.pose.pose.position.y
        battery.z = data.pose.pose.position.z

    if(data.name == "Coke" and data.pose.header.seq == 2):

        coke.x = data.pose.pose.position.x
        coke.y = data.pose.pose.position.y
        coke.z = data.pose.pose.position.z

    if(data.name == "Glue" and data.pose.header.seq == 3):

        glue.x = data.pose.pose.position.x
        glue.y = data.pose.pose.position.y
        glue.z = data.pose.pose.position.z


class Ur5Moveit:

    # Constructor
    def __init__(self):
        global arm_group, hand_group

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm_controller"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        arm_group = moveit_commander.MoveGroupCommander(self._planning_group)
        hand_group = moveit_commander.MoveGroupCommander("grippper_controller")
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        _planning_frame = arm_group.get_planning_frame()
        _eef_link = hand_group.get_end_effector_link()
        _group_names = self._robot.get_group_names()
        _box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(_planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(_eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(_group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        pose_values = arm_group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

    def set_joint_angles(self, arg_list_joint_angles):

        flag_plan = False

        while(flag_plan != True):

            list_joint_values = arm_group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
            # rospy.loginfo(list_joint_values)

            arm_group.set_joint_value_target(arg_list_joint_angles)
            arm_group.plan()
            flag_plan = arm_group.go(wait=True)

            list_joint_values = arm_group.get_current_joint_values()
            # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
            # rospy.loginfo(list_joint_values)

            pose_values = arm_group.get_current_pose().pose
            # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
            # rospy.loginfo(pose_values)

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_pose(self, arg_pose):

        flag_plan = False

        while(flag_plan != True):

            pose_values = arm_group.get_current_pose().pose
            # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
            # rospy.loginfo(pose_values)

            arm_group.set_pose_target(arg_pose)
            flag_plan = arm_group.go(wait=True)  # wait=False for Async Move

            pose_values = arm_group.get_current_pose().pose
            # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
            # rospy.loginfo(pose_values)

            list_joint_values = arm_group.get_current_joint_values()
            # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
            # rospy.loginfo(list_joint_values)

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_gripper_angles(self, arg_list_joint_angles):

        flag_plan = False

        while(flag_plan != True):

            list_joint_values = hand_group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

            hand_group.set_joint_value_target(arg_list_joint_angles)
            hand_group.plan()
            flag_plan = hand_group.go(wait=True)

            list_joint_values = hand_group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        #  rospy.loginfo(list_joint_values)

            pose_values = hand_group.get_current_pose().pose
        #  rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        #  rospy.loginfo(pose_values)

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    armStart = geometry_msgs.msg.Pose()
    armStart.position.x = 0.423340
    armStart.position.y = -0.102573
    armStart.position.z = 1.142720
    armStart.orientation.x = -0.00504774
    armStart.orientation.y = 0.94579767
    armStart.orientation.z = -0.32419815
    armStart.orientation.w = 0.01835316

    armDrop = [math.radians(-11),
               math.radians(14),
               math.radians(-18),
               math.radians(-116),
               math.radians(-91),
               math.radians(79)]

    close = [math.radians(13)]
    close3 = [math.radians(14)]
    close2 = [math.radians(16)]
    openz = [math.radians(0)]

    print "getting coords"
    listener_coord()

    print "Openning grippper"
    ur5.set_gripper_angles(openz)

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = coke.x
    ur5_pose_1.position.y = coke.y - 0.25
    ur5_pose_1.position.z = coke.z + 0.2
    ur5_pose_1.orientation.x = -0.00504774
    ur5_pose_1.orientation.y = 0.94579767
    ur5_pose_1.orientation.z = -0.32419815
    ur5_pose_1.orientation.w = 0.01835316

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = coke.x
    ur5_pose_2.position.y = coke.y - 0.14
    ur5_pose_2.position.z = coke.z + 0.14
    ur5_pose_2.orientation.x = -0.00504774
    ur5_pose_2.orientation.y = 0.94579767
    ur5_pose_2.orientation.z = -0.32419815
    ur5_pose_2.orientation.w = 0.01835316

    ur5_pose_4 = geometry_msgs.msg.Pose()
    ur5_pose_4.position.x = battery.x
    ur5_pose_4.position.y = battery.y - 0.25
    ur5_pose_4.position.z = battery.z + 0.18
    ur5_pose_4.orientation.x = -0.00504774
    ur5_pose_4.orientation.y = 0.94579767
    ur5_pose_4.orientation.z = -0.32419815
    ur5_pose_4.orientation.w = 0.01835316

    ur5_pose_5 = geometry_msgs.msg.Pose()
    ur5_pose_5.position.x = battery.x
    ur5_pose_5.position.y = battery.y - 0.17
    ur5_pose_5.position.z = battery.z + 0.15
    ur5_pose_5.orientation.x = -0.00504774
    ur5_pose_5.orientation.y = 0.94579767
    ur5_pose_5.orientation.z = -0.32419815
    ur5_pose_5.orientation.w = 0.01835316

    ur5_pose_7 = geometry_msgs.msg.Pose()
    ur5_pose_7.position.x = glue.x
    ur5_pose_7.position.y = glue.y - 0.25
    ur5_pose_7.position.z = glue.z + 0.2
    ur5_pose_7.orientation.x = -0.00504774
    ur5_pose_7.orientation.y = 0.94579767
    ur5_pose_7.orientation.z = -0.32419815
    ur5_pose_7.orientation.w = 0.01835316

    ur5_pose_8 = geometry_msgs.msg.Pose()
    ur5_pose_8.position.x = glue.x
    ur5_pose_8.position.y = glue.y - 0.17
    ur5_pose_8.position.z = glue.z + 0.16
    ur5_pose_8.orientation.x = -0.00504774
    ur5_pose_8.orientation.y = 0.94579767
    ur5_pose_8.orientation.z = -0.32419815
    ur5_pose_8.orientation.w = 0.01835316

    print "getting some coke"
    ur5.go_to_pose(ur5_pose_1)
    ur5.go_to_pose(ur5_pose_2)

    print "grabbing the coke"
    ur5.set_gripper_angles(close)
    rospy.sleep(2)

    # print "dropping the coke"
    # ur5.set_joint_angles(armDrop)
    # ur5.set_gripper_angles(openz)
    # rospy.sleep(2)

    # ur5.go_to_pose(armStart)
    # print "getting battery"
    # ur5.go_to_pose(ur5_pose_4)
    # ur5.go_to_pose(ur5_pose_5)

    # print "grabbing the battery"
    # ur5.set_gripper_angles(close2)
    # rospy.sleep(2)

    # print "dropping the battery"
    # ur5.set_joint_angles(armDrop)
    # ur5.set_gripper_angles(openz)
    # rospy.sleep(2)

    # ur5.go_to_pose(armStart)
    # print "getting glue"
    # ur5.go_to_pose(ur5_pose_7)
    # ur5.go_to_pose(ur5_pose_8)

    # print "grabbing the glue"
    # ur5.set_gripper_angles(close3)
    # rospy.sleep(2)

    # print "dropping the glue"
    # ur5.set_joint_angles(armDrop)
    # ur5.set_gripper_angles(openz)

    del ur5


if __name__ == '__main__':
    main()
