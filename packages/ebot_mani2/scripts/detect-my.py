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
        #rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

    def set_joint_angles(self, arg_list_joint_angles):
        list_joint_values = arm_group.get_current_joint_values()
       # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        arm_group.set_joint_value_target(arg_list_joint_angles)
        arm_group.plan()
        flag_plan = arm_group.go(wait=True)

        list_joint_values = arm_group.get_current_joint_values()
        #rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = arm_group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_pose(self, arg_pose):

        pose_values = arm_group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        arm_group.set_pose_target(arg_pose)
        flag_plan = arm_group.go(wait=True)  # wait=False for Async Move

        pose_values = arm_group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = arm_group.get_current_joint_values()
        #rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_gripper_angles(self, arg_list_joint_angles):

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

    # ur5_pose_1.orientation.x = -0.00863625
    # ur5_pose_1.orientation.y = 0.97697319
    # ur5_pose_1.orientation.z = -0.21252023
    # ur5_pose_1.orientation.w = 0.01685081

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

    close = [math.radians(14)]
    close2 = [math.radians(12.5)]
    close3 = [math.radians(9)]
    open_arm = [math.radians(0)]
    open2 = [math.radians(8)]

    print "arm going to start pose"
    ur5.go_to_pose(armStart)
    rospy.sleep(1)

    print "getting coords"
    tfbuffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tfbuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Replace 'object_43' with appropriate name.
            trans1 = tfbuffer.lookup_transform(
                'ebot_base', 'object_43', rospy.Time())
            print "Battery:"
            # object_1 is a vector containing x, y, z values of object 1.
            # object_1.x gives x coordinate of object 1.
            battery = trans1.transform.translation
            print battery
            trans2 = tfbuffer.lookup_transform(
                'ebot_base', 'object_44', rospy.Time())
            print "Coke:"
            coke = trans2.transform.translation
            print coke
            trans3 = tfbuffer.lookup_transform(
                'ebot_base', 'object_45', rospy.Time())
            glue = trans3.transform.translation
            print "Glue"
            print glue

            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print "Error"
    ur5.set_gripper_angles(open_arm)

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = coke.x
    ur5_pose_1.position.y = coke.y - 0.35
    ur5_pose_1.position.z = coke.z
    ur5_pose_1.orientation.x = 0.0034555896355
    ur5_pose_1.orientation.y = -0.999518675798
    ur5_pose_1.orientation.z = 2.45942024614e-05
    ur5_pose_1.orientation.w = 0.0308297750068

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = coke.x
    ur5_pose_2.position.y = coke.y - 0.31
    ur5_pose_2.position.z = coke.z - 0.012
    ur5_pose_2.orientation.x = 0.0034555896355
    ur5_pose_2.orientation.y = -0.999518675798
    ur5_pose_2.orientation.z = 2.45942024614e-05
    ur5_pose_2.orientation.w = 0.0308297750068

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = coke.x
    ur5_pose_3.position.y = coke.y - 0.21
    ur5_pose_3.position.z = coke.z - 0.012
    ur5_pose_3.orientation.x = 0.0034555896355
    ur5_pose_3.orientation.y = -0.999518675798
    ur5_pose_3.orientation.z = 2.45942024614e-05
    ur5_pose_3.orientation.w = 0.0308297750068

    ur5_pose_4 = geometry_msgs.msg.Pose()
    ur5_pose_4.position.x = battery.x
    ur5_pose_4.position.y = battery.y - 0.35
    ur5_pose_4.position.z = battery.z
    ur5_pose_4.orientation.x = 0.0034555896355
    ur5_pose_4.orientation.y = -0.999518675798
    ur5_pose_4.orientation.z = 2.45942024614e-05
    ur5_pose_4.orientation.w = 0.0308297750068

    ur5_pose_5 = geometry_msgs.msg.Pose()
    ur5_pose_5.position.x = battery.x
    ur5_pose_5.position.y = battery.y - 0.31
    ur5_pose_5.position.z = battery.z - 0.012
    ur5_pose_5.orientation.x = 0.0034555896355
    ur5_pose_5.orientation.y = -0.999518675798
    ur5_pose_5.orientation.z = 2.45942024614e-05
    ur5_pose_5.orientation.w = 0.0308297750068

    ur5_pose_6 = geometry_msgs.msg.Pose()
    ur5_pose_6.position.x = battery.x
    ur5_pose_6.position.y = battery.y - 0.22
    ur5_pose_6.position.z = battery.z - 0.012
    ur5_pose_6.orientation.x = 0.0034555896355
    ur5_pose_6.orientation.y = -0.999518675798
    ur5_pose_6.orientation.z = 2.45942024614e-05
    ur5_pose_6.orientation.w = 0.0308297750068

    ur5_pose_7 = geometry_msgs.msg.Pose()
    ur5_pose_7.position.x = glue.x
    ur5_pose_7.position.y = glue.y - 0.35
    ur5_pose_7.position.z = glue.z
    ur5_pose_7.orientation.x = 0.0034555896355
    ur5_pose_7.orientation.y = -0.999518675798
    ur5_pose_7.orientation.z = 2.45942024614e-05
    ur5_pose_7.orientation.w = 0.0308297750068

    ur5_pose_8 = geometry_msgs.msg.Pose()
    ur5_pose_8.position.x = glue.x
    ur5_pose_8.position.y = glue.y - 0.31
    ur5_pose_8.position.z = glue.z - 0.012
    ur5_pose_8.orientation.x = 0.0034555896355
    ur5_pose_8.orientation.y = -0.999518675798
    ur5_pose_8.orientation.z = 2.45942024614e-05
    ur5_pose_8.orientation.w = 0.0308297750068

    ur5_pose_9 = geometry_msgs.msg.Pose()
    ur5_pose_9.position.x = glue.x
    ur5_pose_9.position.y = glue.y - 0.21
    ur5_pose_9.position.z = glue.z - 0.012
    ur5_pose_9.orientation.x = 0.0034555896355
    ur5_pose_9.orientation.y = -0.999518675798
    ur5_pose_9.orientation.z = 2.45942024614e-05
    ur5_pose_9.orientation.w = 0.0308297750068

    print ur5_pose_1

    print "getting some coke"
    ur5.go_to_pose(ur5_pose_1)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_2)
    rospy.sleep(2)
    ur5.go_to_pose(ur5_pose_3)
    rospy.sleep(1)
    ur5.set_gripper_angles(close)
    rospy.sleep(1)
    print "dropping the coke"
    ur5.set_joint_angles(armDrop)

    ur5.set_gripper_angles(open_arm)
    rospy.sleep(5)
    ur5.set_gripper_angles(open2)

    ur5.go_to_pose(armStart)
    print "getting battery"
    ur5.go_to_pose(ur5_pose_4)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_5)
    rospy.sleep(1.5)
    ur5.go_to_pose(ur5_pose_6)
    rospy.sleep(1)
    ur5.set_gripper_angles(close2)
    rospy.sleep(2)
    print "dropping the battery"
    ur5.set_joint_angles(armDrop)

    ur5.set_gripper_angles(open_arm)
    rospy.sleep(4)

    ur5.go_to_pose(armStart)
    print "getting glue"
    ur5.go_to_pose(ur5_pose_7)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_8)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_9)
    rospy.sleep(1)
    ur5.set_gripper_angles(close3)
    rospy.sleep(1)
    print "dropping the glue"
    ur5.set_joint_angles(armDrop)

    ur5.set_gripper_angles(open_arm)
    rospy.sleep(2)
    del ur5


if __name__ == '__main__':
    main()
