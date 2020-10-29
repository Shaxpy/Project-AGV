#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math


def odom_callback(msg):
    global pose
    global x
    global y
    global theta

    x = round(msg.pose.pose.position.x, 1)
    y = round(msg.pose.pose.position.y, 1)
    rot_q = msg.pose.pose.orientation
    pose = (roll, pitch, th) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    theta = round(th, 1)


def main():
    global pub
    global msg
    msg = Twist()
    rospy.init_node('reading_laser')
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()


def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:144]), 20),
        'fright': min(min(msg.ranges[145:289]), 20),
        'front':  min(min(msg.ranges[290:434]), 20),
        'fleft':  min(min(msg.ranges[435:579]), 20),
        'left':   min(min(msg.ranges[580:720]), 20),
    }
    # rospy.loginfo(regions)
    obstacle(regions)


def obstacle(regions):
    linear_x = 0
    angular_z = 0
    d = 1.2
    if regions['front'] < d and regions['fleft'] < d:
        linear_x = 0
        angular_z = 0.55
    # right obtacle far away
    elif regions['fright'] > d:
        linear_x = 0.25
        angular_z = -0.6
    # right obstacle very close
    elif regions['fright'] < d:
        linear_x = 0.25
        angular_z = 0.6
    # rigjt object very very far
    elif (regions['fright'] > (1.1)*d) and (regions['fleft'] > 5*d):
        linear_x = 0.4
        angular_z = -0.83

    # go_to_waypoint(12.5, 0)
    # rospy.loginfo(regions)
    # rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


if __name__ == '__main__':
    main()

