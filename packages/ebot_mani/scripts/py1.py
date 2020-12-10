#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


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

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(_planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(_eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(_group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = arm_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        arm_group.set_pose_target(arg_pose)
        flag_plan = arm_group.go(wait=True)  # wait=False for Async Move

        pose_values = arm_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = arm_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        hand_group.set_named_target(arg_pose_name)
        plan = hand_group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo(
            '\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    # left up
    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = 0.463088
    ur5_pose_2.position.y = 0.226275
    ur5_pose_2.position.z = 1.000000
    ur5_pose_2.orientation.x = -0.30060515
    ur5_pose_2.orientation.y = -0.63710335
    ur5_pose_2.orientation.z = 0.63818203
    ur5_pose_2.orientation.w = 0.31057938

    # left down
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.463088
    ur5_pose_1.position.y = 0.226275
    ur5_pose_1.position.z = 0.870000
    ur5_pose_1.orientation.x = -0.30060515
    ur5_pose_1.orientation.y = -0.63710335
    ur5_pose_1.orientation.z = 0.63818203
    ur5_pose_1.orientation.w = 0.31057938

    # blue box
    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.037620
    ur5_pose_3.position.y = -0.650827
    ur5_pose_3.position.z = 1.100000
    ur5_pose_3.orientation.x = -0.20201409
    ur5_pose_3.orientation.y = 0.66867615
    ur5_pose_3.orientation.z = -0.69186887
    ur5_pose_3.orientation.w = 0.18270189

# mid up
    ur5_pose_4 = geometry_msgs.msg.Pose()
    ur5_pose_4.position.x = 0.585464
    ur5_pose_4.position.y = 0.004003
    ur5_pose_4.position.z = 0.966628
    ur5_pose_4.orientation.x = -0.26785803
    ur5_pose_4.orientation.y = -0.65835475
    ur5_pose_4.orientation.z = 0.65744176
    ur5_pose_4.orientation.w = 0.25018281

# mid down
    ur5_pose_5 = geometry_msgs.msg.Pose()
    ur5_pose_5.position.x = 0.585464
    ur5_pose_5.position.y = 0.004003
    ur5_pose_5.position.z = 0.850000
    ur5_pose_5.orientation.x = -0.26785803
    ur5_pose_5.orientation.y = -0.65835475
    ur5_pose_5.orientation.z = 0.65744176
    ur5_pose_5.orientation.w = 0.25018281

# red box
    ur5_pose_6 = geometry_msgs.msg.Pose()
    ur5_pose_6.position.x = -0.081268
    ur5_pose_6.position.y = 0.690783
    ur5_pose_6.position.z = 1.100000
    ur5_pose_6.orientation.x = -0.31539807
    ur5_pose_6.orientation.y = -0.65470315
    ur5_pose_6.orientation.z = 0.61738495
    ur5_pose_6.orientation.w = 0.3012037

# right up
    ur5_pose_7 = geometry_msgs.msg.Pose()
    ur5_pose_7.position.x = 0.549341
    ur5_pose_7.position.y = -0.255975
    ur5_pose_7.position.z = 1.000000
    ur5_pose_7.orientation.x = -0.70026215
    ur5_pose_7.orientation.y = 0.02524794
    ur5_pose_7.orientation.z = -0.05794208
    ur5_pose_7.orientation.w = 0.71108239

# right down
    ur5_pose_8 = geometry_msgs.msg.Pose()
    ur5_pose_8.position.x = 0.549341
    ur5_pose_8.position.y = -0.255975
    ur5_pose_8.position.z = 0.881281
    ur5_pose_8.orientation.x = -0.70026215
    ur5_pose_8.orientation.y = 0.02524794
    ur5_pose_8.orientation.z = -0.05794208
    ur5_pose_8.orientation.w = 0.71108239
    # while not rospy.is_shutdown():
    ur5.go_to_pose(ur5_pose_2)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_1)
    rospy.sleep(1)
#    ur5.go_to_pose(ur5_pose_2)
#    rospy.sleep(1)
    ur5.go_to_predefined_pose("close0.2")
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_2)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_3)
    rospy.sleep(1)
    ur5.go_to_predefined_pose("close0.05")
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_4)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_5)
    rospy.sleep(1)
    ur5.go_to_predefined_pose("close0.25")
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_4)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_6)
    rospy.sleep(1)
    ur5.go_to_predefined_pose("close0.05")
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_7)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_8)
    rospy.sleep(1)
    ur5.go_to_predefined_pose("close0.25")
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_7)
    rospy.sleep(1)
    ur5.go_to_pose(ur5_pose_3)
    rospy.sleep(1)
    ur5.go_to_predefined_pose("close0.05")
    rospy.sleep(1)


if __name__ == '__main__':
    main()
