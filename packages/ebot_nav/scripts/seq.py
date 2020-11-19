#!/usr/bin/env python2.7

import rospy
# For using SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def client_coordinates(x, y):

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

    goal.target_pose.pose.orientation.w = 2.0

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


def waypoints():
        # Rospy node for SimpleActionClient to publish and subscribe
    rospy.init_node('movebase_client_py')
    points_seq = [-9.1, -1.2, 10.7, 10.5,
                      12.6, -1.81, 18.2, -1.4, -2.0, 4.0]
    for i in [0, 2, 4, 6, 8]:
        result = client_coordinates(points_seq[i], points_seq[i+1])
    return result


# Python node executed as main process
if __name__ == '__main__':
    try:
        # Assigning the waypoints to the Ebot mentioned in Task's problem statement
        result = waypoints()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

