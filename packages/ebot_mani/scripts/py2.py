#! /usr/bin/env python

# Importing Libraries
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

# Creating Ur5Moveit class


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

    # A function to set the joint angles of the arm
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = arm_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        arm_group.set_joint_value_target(arg_list_joint_angles)
        arm_group.plan()
        flag_plan = arm_group.go(wait=True)

        list_joint_values = arm_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = arm_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # A function to set the arm's position to a predefined pose
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

# Main Function


def main():

    # Instantiating the Ur5Moveit class
    ur5 = Ur5Moveit()

    # Creating arrays to be fed into the set_joint_angles function to set the joint angles

    leftUp = [math.radians(13),
              math.radians(-53),
              math.radians(86),
              math.radians(56),
              math.radians(88),
              math.radians(-25)]

    leftDown = [math.radians(13),
                math.radians(-47),
                math.radians(88),
                math.radians(49),
                math.radians(88),
                math.radians(-24)]

    blueBox = [math.radians(97),
               math.radians(-124),
               math.radians(-54),
               math.radians(266),
               math.radians(88),
               math.radians(-301)]

    midUp = [math.radians(188),
             math.radians(-107),
             math.radians(-98),
             math.radians(292),
             math.radians(92),
             math.radians(-210)]

    midDown = [math.radians(188),
               math.radians(-124),
               math.radians(-109),
               math.radians(321),
               math.radians(92),
               math.radians(-210)]

    redBox = [math.radians(274),
              math.radians(-124),
              math.radians(-66),
              math.radians(282),
              math.radians(94),
              math.radians(-124)]

    rightUp = [math.radians(167),
               math.radians(-114),
               math.radians(-88),
               math.radians(289),
               math.radians(91),
               math.radians(-279)]

    rightDown = [math.radians(167),
                 math.radians(-124),
                 math.radians(-99),
                 math.radians(308),
                 math.radians(91),
                 math.radians(-279)]

    blueBox2 = [math.radians(102),
                math.radians(-117),
                math.radians(-68),
                math.radians(273),
                math.radians(87),
                math.radians(-344)]

    # Actions for picking and placing object 2
    ur5.set_joint_angles(leftUp)
    ur5.set_joint_angles(leftDown)
    ur5.go_to_predefined_pose("close0.2")
    rospy.sleep(1)
    ur5.set_joint_angles(leftUp)
    ur5.set_joint_angles(blueBox)
    ur5.go_to_predefined_pose("close0.05")
    rospy.sleep(1)

    # Actions for picking and placing object 1
    ur5.set_joint_angles(midUp)
    ur5.set_joint_angles(midDown)
    ur5.go_to_predefined_pose("close0.2")
    rospy.sleep(1)
    ur5.set_joint_angles(midUp)
    ur5.set_joint_angles(redBox)
    ur5.go_to_predefined_pose("close0.05")
    rospy.sleep(1)

    # Actions for picking and placing object 3
    ur5.set_joint_angles(rightUp)
    ur5.set_joint_angles(rightDown)
    ur5.go_to_predefined_pose("close0.2")
    rospy.sleep(1)
    ur5.set_joint_angles(rightUp)
    ur5.set_joint_angles(blueBox2)
    ur5.go_to_predefined_pose("close0.05")
    rospy.sleep(1)

    # Destroying the Ur5Moveit class
    del ur5


# Calling the main function
if __name__ == '__main__':
    main()
