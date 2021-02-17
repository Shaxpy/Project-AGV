#!/usr/bin/env python2.7

#Making the necessary imports
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import roslib
import time
import roslaunch
import tf2_ros
from math import pi
from object_msgs.msg import ObjectPose
from std_srvs.srv import Empty

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

    # A method to directed the end effector of the arm to given coordinates

    def go_to_uni_pose(self, arg_pose):
        tfbuffer = tf2_ros.Buffer()

        listener = tf2_ros.TransformListener(tfbuffer)
        rate = rospy.Rate(10.0)

        flag_plan = False

        alpha = geometry_msgs.msg.Pose()
        alpha = arg_pose
        while not rospy.is_shutdown():
            try:
                trans = tfbuffer.lookup_transform(
                    'odom', 'ebot_base', rospy.Time())

                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                continue
        alpha.position.x = arg_pose.position.x-trans.transform.translation.x
        alpha.position.y = arg_pose.position.y-trans.transform.translation.y

        return alpha

    # A method to set the joint angles of the arm
    def set_joint_angles(self, arg_list_joint_angles):

        flag_plan = False
        count = 0

        while(count < 15):

            list_joint_values = arm_group.get_current_joint_values()
            arm_group.set_joint_value_target(arg_list_joint_angles)
            arm_group.plan()
            flag_plan = arm_group.go(wait=True)

            list_joint_values = arm_group.get_current_joint_values()

            pose_values = arm_group.get_current_pose().pose

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
                break
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed. Trying again" + '\033[0m')
                count += 1

        return flag_plan
# Method to calculate position with respect to odom
    def go_to_pose(self, arg_pose):

        flag_plan = False
        count = 0

        while(count < 15):

            pose_values = arm_group.get_current_pose().pose

            arm_group.set_pose_target(arg_pose)
            flag_plan = arm_group.go(wait=True)

            pose_values = arm_group.get_current_pose().pose

            list_joint_values = arm_group.get_current_joint_values()

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
                break
            else:
                rospy.logerr(
                    '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found. Trying again" + '\033[0m')
                count += 1

        return flag_plan

    # A method to set the gripping angle of the gripper
    def set_gripper_angles(self, arg_list_joint_angles):

        flag_plan = False
        count = 0

        while(count < 15):

            list_joint_values = hand_group.get_current_joint_values()

            hand_group.set_joint_value_target(arg_list_joint_angles)
            hand_group.plan()
            flag_plan = hand_group.go(wait=True)

            list_joint_values = hand_group.get_current_joint_values()

            pose_values = hand_group.get_current_pose().pose

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
                break
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed. Trying again" + '\033[0m')
                count += 1

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

# Localization using TF2

def object_coordinates(object_number):
    measure1 = time.time()
    tfbuffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tfbuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            if(time.time() - measure1 > 6.00):
                object = 0.0
                print("time done")
                break
            # TF2 transform for getting localization coordinates
            trans1 = tfbuffer.lookup_transform(
                'ebot_base', object_number[0], rospy.Time())
             # object_1 is a vector containing x, y, z values of object 1.
            # object_1.x gives x coordinate of object 1.
            object = trans1.transform.translation
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            continue
    return object

# Launching Perception stack

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
    uuid, ['/home/akhiljayan29aj/catkin_ws1/src/object_recognition/launch/meet.launch'])

uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid1)
launch1 = roslaunch.parent.ROSLaunchParent(
    uuid1, ['/home/akhiljayan29aj/catkin_ws1/src/object_recognition/launch/pantry.launch'])

uuid0 = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid0)
launch0 = roslaunch.parent.ROSLaunchParent(
    uuid0, ['/home/akhiljayan29aj/catkin_ws1/src/object_recognition/launch/pantry.launch'])

uuid2 = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid2)
launch2 = roslaunch.parent.ROSLaunchParent(
    uuid2, ['/home/akhiljayan29aj/catkin_ws1/src/object_recognition/launch/store.launch'])


# Python node executed as main process
if __name__ == '__main__':
    try:
        rospy.init_node('taskfive', anonymous=True)

        ur5 = Ur5Moveit()
        rospy.wait_for_service('/clear_octomap')
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

        rospy.loginfo(
            '\033[94m' + "Started Run!" + '\033[0m')

        # Directions
        north = [0.0, 1.0]
        south = [1.0, 0.0]
        east = [-0.7, 0.7]
        west = [0.7, 0.7]
        northeast = [-3.16771732e-01, 9.48501803e-01]
        southwest = [0.949, 0.315]

        # Arm pose for better movement
        movearm = [math.radians(-22),
                   math.radians(-25),
                   math.radians(20),
                   math.radians(-25),
                   math.radians(-29),
                   math.radians(-17)]


        # # Left table arm pose
        pantry = geometry_msgs.msg.Pose()
        pantry.position.x = 14.428034 + 0.235442
        pantry.position.y = -0.935587 - 0.118663
        pantry.position.z = 1.388446
        pantry.orientation.x = -0.61162194
        pantry.orientation.y = -0.69264205
        pantry.orientation.z = 0.23878188
        pantry.orientation.w = 0.29857799

        # right table arm pose
        pantry2 = geometry_msgs.msg.Pose()
        pantry2.position.x = 11.448368 + 0.235442
        pantry2.position.y = -0.951783 - 0.118663
        pantry2.position.z = 1.388446
        pantry2.orientation.x = -0.61162194
        pantry2.orientation.y = -0.69264205
        pantry2.orientation.z = 0.23878188
        pantry2.orientation.w = 0.29857799

        # uni Meeting room obj
        meeting = geometry_msgs.msg.Pose()
        meeting.position.x = 7.651076 + 0.292582
        meeting.position.y = 2.545992 + 0.109048
        meeting.position.z = 0.756314
        meeting.orientation.x = -0.00174895
        meeting.orientation.y = -0.99997116
        meeting.orientation.z = 0.00113101
        meeting.orientation.w = -0.00730297

        # Left store arm pose
        store = geometry_msgs.msg.Pose()
        store.position.x = 25.806717 + 0.100052
        store.position.y = -3.050722 + 0.430078
        store.position.z = 0.857543
        store.orientation.x = 0.66333574
        store.orientation.y = 0.74296358
        store.orientation.z = 0.06133693
        store.orientation.w = 0.06502766

        # right store arm pose
        store2 = geometry_msgs.msg.Pose()
        store2.position.x = 25.806717 + 0.063176
        store2.position.y = -3.050722 - 0.227713
        store2.position.z = 1.031617
        store2.orientation.x = -0.67577561
        store2.orientation.y = -0.73706646
        store2.orientation.z = 0.00494999
        store2.orientation.w = 0.00598872

        # Left store arm pose
        store3 = geometry_msgs.msg.Pose()
        store3.position.x = 25.806717 + 0.100052
        store3.position.y = -3.050722 + 0.430078
        store3.position.z = 0.857543
        store3.orientation.x = 0.66333574
        store3.orientation.y = 0.74296358
        store3.orientation.z = 0.06133693
        store3.orientation.w = 0.06502766

        # right store arm pose
        store4 = geometry_msgs.msg.Pose()
        store4.position.x = 25.806717 + 0.063176
        store4.position.y = -3.050722 - 0.227713
        store4.position.z = 1.031617
        store4.orientation.x = -0.67577561
        store4.orientation.y = -0.73706646
        store4.orientation.z = 0.00494999
        store4.orientation.w = 0.00598872

        # Gripper joint values
        openz = [math.radians(0)]
        close = [math.radians(17)]
        close1 = [math.radians(10)]
        close2 = [math.radians(17)]

        # Meeting room drop box
        drop1 = geometry_msgs.msg.Pose()
        drop1.position.x = 6.979429 + 0.024911
        drop1.position.y = 2.700901 + 0.408946
        drop1.position.z = 1.115345
        drop1.orientation.x = 0.00751393
        drop1.orientation.y = -0.96497293
        drop1.orientation.z = 0.00178604
        drop1.orientation.w = 0.26223578

        # Research room drop box
        drop2 = geometry_msgs.msg.Pose()
        drop2.position.x = 10.800876 - 0.112948
        drop2.position.y = 9.459820 + 0.436382
        drop2.position.z = 1.118052
        drop2.orientation.x = 0.03059092
        drop2.orientation.y = 0.98850601
        drop2.orientation.z = 0.10453721
        drop2.orientation.w = 0.10484288

        # conf room drop box
        drop3 = geometry_msgs.msg.Pose()
        drop3.position.x = 4.940466 + 0.101856
        drop3.position.y = -1.057005 + 0.380650
        drop3.position.z = 1.241397
        drop3.orientation.x = 0.01141813
        drop3.orientation.y = 0.98895515
        drop3.orientation.z = 0.06390285
        drop3.orientation.w = 0.13324328

        # For better movement
        ur5.set_joint_angles(movearm)

        # pantry outside
        client_coordinates(12.96, 1.27, east)

        # # pantry inside
        client_coordinates(13.12, -1.0, north)

        rospy.loginfo(
            '\033[94m' + "Pantry Reached." + '\033[0m')

        # pantry left table
        client_coordinates(14.6, -1.0, north)

        ur5.go_to_pose(ur5.go_to_uni_pose(pantry))

        launch1.start()

        time.sleep(15)

        launch1.shutdown()

        coke = object_coordinates(['object_201'])

        if (coke == 0.0):
            coke = object_coordinates(['object_203'])
            if (coke == 0.0):
                coke = object_coordinates(['object_202'])

        if (coke == 0.0):

            # For better movement
            ur5.set_joint_angles(movearm)

            # # Orienting bot in pantry's left table
            client_coordinates(14.6, -1.0, south)

            # Coordinates of pantry right table
            client_coordinates(11.3, -0.95, [-9.99946462e-01, 1.03476422e-02])
            clear_octomap()
            rospy.sleep(3)

            # # PANTRY 2
            ur5.go_to_pose(ur5.go_to_uni_pose(pantry2))

            launch0.start()

            time.sleep(15)

            launch0.shutdown()

            coke = object_coordinates(['object_208'])
            if (coke == 0.0):
                coke = object_coordinates(['object_209'])

            if (coke != 0):

                ur5_pose_1 = geometry_msgs.msg.Pose()
                ur5_pose_1.position.x = coke.x - 0.23
                ur5_pose_1.position.y = coke.y + 0.003
                ur5_pose_1.position.z = coke.z + 0.18
                ur5_pose_1.orientation.x = -0.61162194
                ur5_pose_1.orientation.y = -0.69264205
                ur5_pose_1.orientation.z = 0.23878188
                ur5_pose_1.orientation.w = 0.29857799

                ur5_pose_2 = geometry_msgs.msg.Pose()
                ur5_pose_2.position.x = coke.x - 0.14
                ur5_pose_2.position.y = coke.y + 0.003
                ur5_pose_2.position.z = coke.z + 0.14
                ur5_pose_2.orientation.x = -0.61162194
                ur5_pose_2.orientation.y = -0.69264205
                ur5_pose_2.orientation.z = 0.23878188
                ur5_pose_2.orientation.w = 0.29857799

                ur5_pose_3 = geometry_msgs.msg.Pose()
                ur5_pose_3.position.x = coke.x - 0.32
                ur5_pose_3.position.y = coke.y - 0.17
                ur5_pose_3.position.z = coke.z + 0.30
                ur5_pose_3.orientation.x = -0.61162194
                ur5_pose_3.orientation.y = -0.69264205
                ur5_pose_3.orientation.z = 0.23878188
                ur5_pose_3.orientation.w = 0.29857799

                ur5.go_to_pose(ur5_pose_1)
                ur5.go_to_pose(ur5_pose_2)

                ur5.set_gripper_angles(close)
                rospy.sleep(2)
                ur5.go_to_pose(ur5_pose_3)

                rospy.sleep(1)

                rospy.loginfo(
                    '\033[94m' + "Coke Picked" + '\033[0m')

                ur5.set_joint_angles(movearm)

            # For better movement
            ur5.set_joint_angles(movearm)

            #Turns bot in pantry 
            client_coordinates(11.35, -0.95, north)

        else:

            ur5_pose_1 = geometry_msgs.msg.Pose()
            ur5_pose_1.position.x = coke.x - 0.23
            ur5_pose_1.position.y = coke.y + 0.05
            ur5_pose_1.position.z = coke.z + 0.18
            ur5_pose_1.orientation.x = -0.65849536
            ur5_pose_1.orientation.y = -0.68264492
            ur5_pose_1.orientation.z = 0.22230603
            ur5_pose_1.orientation.w = 0.22574281

            ur5_pose_2 = geometry_msgs.msg.Pose()
            ur5_pose_2.position.x = coke.x - 0.16
            ur5_pose_2.position.y = coke.y + 0.05
            ur5_pose_2.position.z = coke.z + 0.14
            ur5_pose_2.orientation.x = -0.65849536
            ur5_pose_2.orientation.y = -0.68264492
            ur5_pose_2.orientation.z = 0.22230603
            ur5_pose_2.orientation.w = 0.22574281

            ur5_pose_3 = geometry_msgs.msg.Pose()
            ur5_pose_3.position.x = coke.x - 0.32
            ur5_pose_3.position.y = coke.y - 0.17
            ur5_pose_3.position.z = coke.z + 0.30
            ur5_pose_3.orientation.x = -0.65849536
            ur5_pose_3.orientation.y = -0.68264492
            ur5_pose_3.orientation.z = 0.22230603
            ur5_pose_3.orientation.w = 0.22574281

            ur5.go_to_pose(ur5_pose_1)

            ur5.go_to_pose(ur5_pose_2)

            ur5.set_gripper_angles(close)
            rospy.sleep(2)

            ur5.go_to_pose(ur5_pose_3)

            rospy.loginfo(
                '\033[94m' + "Coke Picked" + '\033[0m')

            ur5.set_joint_angles(movearm)

            client_coordinates(14.5, -1.0, west)

            # # pantry left table turn toward south
            client_coordinates(14.5, -1.0, south)

        # # # # # Inside of Pantry
        client_coordinates(13.12, -1.0, west)

        # # # # # Outside of Pantry
        client_coordinates(12.96, 1.27, south)
        clear_octomap()

        # # # # Outside of Meeting Room
        client_coordinates(8.75, 1.09, west)

        # # # # Coordinates of inside of Meeting Room
        client_coordinates(8.75, 2.52, south)
        rospy.loginfo(
            '\033[94m' + "Meeting room Reached" + '\033[0m')

        # # # # # Coordinats of drop box in Meeting Room
        client_coordinates(6.75, 2.5, east)
        client_coordinates(6.75, 2.5, north)
        ur5.set_joint_angles(movearm)
        rospy.sleep(2)
        ur5.go_to_pose(ur5.go_to_uni_pose(drop1))

        ur5.set_gripper_angles(openz)
        rospy.loginfo(
            '\033[94m' + "Coke Dropped in DropBox2" + '\033[0m')

        ur5.set_joint_angles(movearm)
        clear_octomap()

        # Coordinates in Meeting Room
        client_coordinates(7.73, 2.52, [-3.08581007e-02, 9.99523775e-01])

        ur5.go_to_pose(ur5.go_to_uni_pose(meeting))
        clear_octomap()
        #Launch node to detect objects in Meeting Room
        launch.start()

        time.sleep(15)

        launch.shutdown()

        glue = object_coordinates(['object_7'])

        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = glue.x
        ur5_pose_1.position.y = glue.y - 0.23
        ur5_pose_1.position.z = glue.z + 0.18
        ur5_pose_1.orientation.x = -0.00504774
        ur5_pose_1.orientation.y = 0.94579767
        ur5_pose_1.orientation.z = -0.32419815
        ur5_pose_1.orientation.w = 0.01835316

        ur5_pose_2 = geometry_msgs.msg.Pose()
        ur5_pose_2.position.x = glue.x
        ur5_pose_2.position.y = glue.y - 0.17
        ur5_pose_2.position.z = glue.z + 0.16
        ur5_pose_2.orientation.x = -0.00504774
        ur5_pose_2.orientation.y = 0.94579767
        ur5_pose_2.orientation.z = -0.32419815
        ur5_pose_2.orientation.w = 0.01835316

        ur5_pose_3 = geometry_msgs.msg.Pose()
        ur5_pose_3.position.x = glue.x
        ur5_pose_3.position.y = glue.y - 0.23
        ur5_pose_3.position.z = glue.z + 0.30
        ur5_pose_3.orientation.x = -0.00504774
        ur5_pose_3.orientation.y = 0.94579767
        ur5_pose_3.orientation.z = -0.32419815
        ur5_pose_3.orientation.w = 0.01835316

        ur5.go_to_pose(ur5_pose_1)
        ur5.go_to_pose(ur5_pose_2)

        ur5.set_gripper_angles(close)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_3)
        rospy.loginfo(
            '\033[94m' + "Glue Picked" + '\033[0m')

        ur5.set_joint_angles(movearm)
        clear_octomap()

        # # # Coordinates of inside of Meeting Room
        client_coordinates(8.75, 2.52, east)

        # # # # Coordinates of outside of Meeting Room
        client_coordinates(8.75, 1.09, north)

        # # research
        client_coordinates(10.75, 9.48, east)
        rospy.loginfo(
            '\033[94m' + "Research Lab Reached" + '\033[0m')
        ur5.set_joint_angles(movearm)

        ur5.go_to_pose(ur5.go_to_uni_pose(drop2))

        ur5.set_gripper_angles(openz)
        rospy.loginfo(
            '\033[94m' + "Glue Dropped in DropBox3" + '\033[0m')

        ur5.set_joint_angles(movearm)
        clear_octomap()

        #Coordinates of Store
        client_coordinates(25.89, -3.05, northeast)

        rospy.loginfo(
            '\033[94m' + "Store Room Reached" + '\033[0m')

        ur5.go_to_pose(ur5.go_to_uni_pose(store))

        #Launches node to detect objects in Store Room
        
        launch2.start()

        time.sleep(15)

        launch2.shutdown()

        fpga_left = object_coordinates(['object_11'])
        if (fpga_left == 0):
            fpga_left = object_coordinates(['object_10'])

        ur5.go_to_pose(ur5.go_to_uni_pose(store2))

        launch2.start()

        #  Delay of 15 seconds for showing the objects detected!

        time.sleep(15)

        launch2.shutdown()

        fpga_right = object_coordinates(['object_22'])
        if (fpga_right == 0):
            fpga_right = object_coordinates(['object_11'])
            if (fpga_right == 0):
                fpga_right = object_coordinates(['object_10'])

        if(fpga_right == 0):
            ur5.go_to_pose(ur5.go_to_uni_pose(store3))
            ur5_pose_1 = geometry_msgs.msg.Pose()
            ur5_pose_1.position.x = fpga_left.x - 0.265
            ur5_pose_1.position.y = fpga_left.y + 0.165
            ur5_pose_1.position.z = fpga_left.z + 0.16
            ur5_pose_1.orientation.x = -0.82452704
            ur5_pose_1.orientation.y = -0.487313
            ur5_pose_1.orientation.z = 0.13977996
            ur5_pose_1.orientation.w = 0.25128225

            ur5_pose_2 = geometry_msgs.msg.Pose()
            ur5_pose_2.position.x = fpga_left.x - 0.16
            ur5_pose_2.position.y = fpga_left.y + 0.12
            ur5_pose_2.position.z = fpga_left.z + 0.18
            ur5_pose_2.orientation.x = -0.82452704
            ur5_pose_2.orientation.y = -0.487313
            ur5_pose_2.orientation.z = 0.13977996
            ur5_pose_2.orientation.w = 0.25128225

            ur5_pose_3 = geometry_msgs.msg.Pose()
            ur5_pose_3.position.x = fpga_left.x - 0.256
            ur5_pose_3.position.y = fpga_left.y + 0.165
            ur5_pose_3.position.z = fpga_left.z + 0.30
            ur5_pose_3.orientation.x = -0.82452704
            ur5_pose_3.orientation.y = -0.487313
            ur5_pose_3.orientation.z = 0.13977996
            ur5_pose_3.orientation.w = 0.25128225

            ur5.go_to_pose(ur5_pose_1)
            ur5.go_to_pose(ur5_pose_2)

            ur5.set_gripper_angles(close2)
            rospy.sleep(2)
            ur5.go_to_pose(ur5_pose_3)

            rospy.loginfo(
                '\033[94m' + "FPGA Picked" + '\033[0m')

            ur5.set_joint_angles(movearm)

        elif(fpga_left == 0):
            ur5.go_to_pose(ur5.go_to_uni_pose(store4))

            ur5_pose_1 = geometry_msgs.msg.Pose()
            ur5_pose_1.position.x = fpga_right.x - 0.265
            ur5_pose_1.position.y = fpga_right.y + 0.165
            ur5_pose_1.position.z = fpga_right.z + 0.16
            ur5_pose_1.orientation.x = -0.82452704
            ur5_pose_1.orientation.y = -0.487313
            ur5_pose_1.orientation.z = 0.13977996
            ur5_pose_1.orientation.w = 0.25128225

            ur5_pose_2 = geometry_msgs.msg.Pose()
            ur5_pose_2.position.x = fpga_right.x - 0.16
            ur5_pose_2.position.y = fpga_right.y + 0.12
            ur5_pose_2.position.z = fpga_right.z + 0.18
            ur5_pose_2.orientation.x = -0.82452704
            ur5_pose_2.orientation.y = -0.487313
            ur5_pose_2.orientation.z = 0.13977996
            ur5_pose_2.orientation.w = 0.25128225

            ur5_pose_3 = geometry_msgs.msg.Pose()
            ur5_pose_3.position.x = fpga_right.x - 0.265
            ur5_pose_3.position.y = fpga_right.y + 0.165
            ur5_pose_3.position.z = fpga_right.z + 0.30
            ur5_pose_3.orientation.x = -0.82452704
            ur5_pose_3.orientation.y = -0.487313
            ur5_pose_3.orientation.z = 0.13977996
            ur5_pose_3.orientation.w = 0.25128225

            ur5.go_to_pose(ur5_pose_1)
            ur5.go_to_pose(ur5_pose_2)

            ur5.set_gripper_angles(close2)
            rospy.sleep(2)
            ur5.go_to_pose(ur5_pose_3)
            rospy.loginfo(
                '\033[94m' + "FPGA Picked" + '\033[0m')

            ur5.set_joint_angles(movearm)

        client_coordinates(25.8, -3.05, southwest)

        # # # Coordinates of Conference Room  outside
        client_coordinates(5.19, 0.85, east)
        clear_octomap()

        # # # Coordinates of Conference Room inside
        client_coordinates(4.93, -1.18, east)
        rospy.loginfo(
            '\033[94m' + "Conference Room Reached" + '\033[0m')

        ur5.set_joint_angles(movearm)
        rospy.sleep(2)

        ur5.go_to_pose(ur5.go_to_uni_pose(drop3))

        ur5.set_gripper_angles(openz)
        rospy.sleep(2)
        ur5.set_joint_angles(movearm)
        client_coordinates(4.93, -1.18, west)

        client_coordinates(5.19, 0.85, south)
        client_coordinates(0.0, 0.0, south)

        rospy.loginfo(
            '\033[94m' + "FPGA Board Dropped in DropBox1" + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Mission Accomplished!" + '\033[0m')

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
