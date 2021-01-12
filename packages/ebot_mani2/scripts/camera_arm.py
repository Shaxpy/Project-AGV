#! /usr/bin/env python

# Importing libraries
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

# Creating Ur5Moveit class


class Ur5Moveit:

    # Constructor
    def __init__(self):
        global arm_group, hand_group

        rospy.init_node('tf_echo', anonymous=True)

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
        pose_values = arm_group.get_current_pose().pose

    # A method to send the end effector to a given set of coordinates
    def go_to_pose(self, arg_pose):

        flag_plan = False

        while(flag_plan != True):

            pose_values = arm_group.get_current_pose().pose

            arm_group.set_pose_target(arg_pose)
            flag_plan = arm_group.go(wait=True)

            pose_values = arm_group.get_current_pose().pose

            list_joint_values = arm_group.get_current_joint_values()

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

# Main Function


def main():

    # Instantiating the Ur5Moveit class
    ur5 = Ur5Moveit()

    # Creating pose to be fed into the go_to_pose method of Ur5Moveit class

    armStart = geometry_msgs.msg.Pose()
    armStart.position.x = 0.423340
    armStart.position.y = -0.102573
    armStart.position.z = 1.142720
    armStart.orientation.x = -0.0829618
    armStart.orientation.y = 0.95911628
    armStart.orientation.z = -0.27055914
    armStart.orientation.w = -0.00332408

    # Going to start position
    print "arm going to start pose"
    ur5.go_to_pose(armStart)

    del ur5


if __name__ == '__main__':
    # Calling the main function
    main()

    # Creating a publisher handle to publish the object poses to /detection_info
    publisher = rospy.Publisher('/detection_info', ObjectPose, queue_size=10)

    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            obj_msg_1 = ObjectPose()
            obj_msg_2 = ObjectPose()
            obj_msg_3 = ObjectPose()

            # Object => Battery
            trans1 = tfbuffer.lookup_transform(
                'base_link', 'object_43', rospy.Time())
            battery = trans1.transform.translation
            print "Battery:"
            print battery

            obj_msg_1.name = "Battery"
            obj_msg_1.pose.header.seq = 1
            obj_msg_1.pose.header.stamp = rospy.Time.now()
            obj_msg_1.pose.header.frame_id = 'base_link'
            obj_msg_1.pose.pose.position.x = battery.x
            obj_msg_1.pose.pose.position.y = battery.y
            obj_msg_1.pose.pose.position.z = battery.z

            # Object => Coke
            trans2 = tfbuffer.lookup_transform(
                'base_link', 'object_44', rospy.Time())
            coke = trans2.transform.translation
            print "Coke:"
            print coke

            obj_msg_2.name = "Coke"
            obj_msg_2.pose.header.seq = 2
            obj_msg_2.pose.header.stamp = rospy.Time.now()
            obj_msg_2.pose.header.frame_id = 'base_link'
            obj_msg_2.pose.pose.position.x = coke.x
            obj_msg_2.pose.pose.position.y = coke.y
            obj_msg_2.pose.pose.position.z = coke.z

            # Object => Glue
            trans3 = tfbuffer.lookup_transform(
                'base_link', 'object_45', rospy.Time())
            glue = trans3.transform.translation
            print "Glue"
            print glue

            obj_msg_3.name = "Glue"
            obj_msg_3.pose.header.seq = 3
            obj_msg_3.pose.header.stamp = rospy.Time.now()
            obj_msg_3.pose.header.frame_id = 'base_link'
            obj_msg_3.pose.pose.position.x = glue.x
            obj_msg_3.pose.pose.position.y = glue.y
            obj_msg_3.pose.pose.position.z = glue.z

            publisher.publish(obj_msg_1)
            publisher.publish(obj_msg_2)
            publisher.publish(obj_msg_3)

            break

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print "Trying again"
