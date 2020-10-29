#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math


def odom_callback(msg):
    global x
    global y
    global theta

    x = round(msg.pose.pose.position.x, 1)
    y = round(msg.pose.pose.position.y, 1)
    rot_q = msg.pose.pose.orientation
    th = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])[2]
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
    regions = {
        'right':  min(min(msg.ranges[0:144]), 20),
        'fright': min(min(msg.ranges[145:289]), 20),
        'front':  min(min(msg.ranges[290:434]), 20),
        'fleft':  min(min(msg.ranges[435:579]), 20),
        'left':   min(min(msg.ranges[580:720]), 20),
    }
    # rospy.loginfo(regions)
    obstacle(regions)


def goal(final_x, final_y):
    inc_x = final_x - x
    inc_y = final_y - y
    angle_to_goal = round(math.atan2(inc_y, inc_x), 1)
    diff = angle_to_goal-theta
    p = 0.7
    error = p*diff
    theta_prec = math.pi/90
    dist_prec = 0.3
    err_pos = math.sqrt(pow(final_y - y, 2) + pow(final_x - x, 2))
    print "error=="+str(error)
    print "theta=="+str(theta)
    print "angle=="+str(angle_to_goal)

    if (error) > theta_prec:

        msg.linear.x = 0.0
        msg.angular.z = 0.9 + error
    # turn clockwise
    elif (error) < -theta_prec:

        msg.linear.x = 0.0
        msg.angular.z = -0.9 + error

    # move forward
    elif (err_pos > dist_prec):

        msg.linear.x = 0.9
        msg.angular.z = 0.0
    # stop
    elif (err_pos <= dist_prec):

        msg.linear.x = 0.0
        msg.angular.z = 0.0
    pub.publish(msg)


def obstacle(regions):

    linear_x = 0
    angular_z = 0

    state_description = ''
    d = 1.5
    if regions['front'] > d and regions['fleft'] > d and regions['left'] > d and regions['right'] > d:
        state_description = ' Straight'
        linear_x = 0.7
        angular_z = 0
    elif regions['front'] < d and regions['left'] > d and regions['right'] > d and regions['fleft'] < d:
        state_description = ' Left'
        linear_x = 0
        angular_z = 0.55
    elif regions['front'] < d and regions['left'] > d and regions['fright'] < d and regions['fleft'] > d:
        state_description = ' more Left'
        linear_x = 0
        angular_z = 0.57
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = ' left straight'
        linear_x = 0.8
        angular_z = 0
    elif regions['front'] > d and regions['left'] > d and regions['fleft'] > d and regions['right'] < d/1.2 and regions['fright'] > d:
        state_description = ' Rgt straight'
        linear_x = 0.3
        angular_z = -0.68
    elif regions['front'] > d and regions['fleft'] > d and regions['left'] > d and regions['right'] < d:
        state_description = 'Goal'

    else:

        state_description = 'unknown case'
    # go_to_waypoint(12.5, 0)
    rospy.loginfo(regions)
    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


if __name__ == '__main__':
    main()
