#!/usr/bin/env python

# Importing Libraries
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math

# Variables
x, y, theta, final_y, final_x = 0, 0, 0, 0, 12.5

position_, initial = Point(), Point()


# Control Loop - Initialising Loop
def control_loop():
    # Setting Global Variables
    global pub, goal, rate, velocity_msg

    velocity_msg = Twist()
    goal = Point()

    # Initializing Node
    rospy.init_node('ebot_controller')

    # Setting Publisher and Subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Setting Rate
    rate = rospy.Rate(10)

    # Calling Path Tracing Algorithm
    path_tracing(1)

    rospy.spin()

# WayPoint - A function to set the net waypoint
def waypoint(t):
    x = ((math.pi*t)/180)
    y = (2*(math.sin(x))*(math.sin(x/2)))
    return [round(x, 1), round(y, 1)]

# Odometry Callback - A callback function used to extract the information from the subscribed topic
def odom_callback(msg):
    global x, y, theta, position_

    position_ = msg.pose.pose.position
    x = round(msg.pose.pose.position.x, 1)
    y = round(msg.pose.pose.position.y, 1)
    rot_q = msg.pose.pose.orientation
    th = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])[2]
    theta = round(th, 1)

# Laser Callback - A callback function used to extract the information from the subscribed topic
def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:144]), 20),
        'fright': min(min(msg.ranges[145:289]), 20),
        'front':  min(min(msg.ranges[290:434]), 20),
        'fleft':  min(min(msg.ranges[435:579]), 20),
        'left':   min(min(msg.ranges[580:720]), 20),
    }

# Go To Goal: A function to go to a desired point in the x-y plane
def go_to_goal(final_x, final_y):
    # Calculating difference in current and desired points
    inc_x = final_x - x
    inc_y = final_y - y

    # Calculating angle towards the goal
    angle_to_goal = round(math.atan2(inc_y, inc_x), 1)

    # P only Controller
    diff = angle_to_goal-theta
    p = 1.1
    error = p*diff

    # Setting tolerance
    theta_prec = math.pi/90
    dist_prec = 0.3

    # Calculating distance between current and desired points
    err_pos = math.sqrt(pow(final_y - y, 2) + pow(final_x - x, 2))

    # Turning Anti-clockwise
    if (error) > theta_prec:
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.7 + error

    # Turning Clockwise
    elif (error) < -theta_prec:
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = -0.7 + error

    # Moving Forward
    elif (err_pos > dist_prec):
        velocity_msg.linear.x = 0.9
        velocity_msg.angular.z = 0.0

    # Stopping Condition
    elif (err_pos < 0.2):
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0

    # Publishing Angular and Linear Velocities
    pub.publish(velocity_msg)

# Obstacle Avoider - A function to avoid any obstacle in path
def obstacle(regions):
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    d = 1.2
    # front object and left object
    if regions['front'] < d and regions['fleft'] < d :
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.55
        print(regions['front'],regions['fleft'])   
    # right obtacle far away
    elif regions['fright'] > d :
        velocity_msg.linear.x = 0.25
        velocity_msg.angular.z = -0.6
        print(regions['fright'])
    #right obstacle very close 
    elif regions['fright'] < d :
        velocity_msg.linear.x = 0.25
        velocity_msg.angular.z = 0.6
        print(regions['fright'])
    #rigjt object very very far
    elif (regions['fright'] > (1.1)*d) and (regions['fleft'] > 5*d):
        velocity_msg.linear.x = 0.4
        velocity_msg.angular.z = -0.83
        print(regions['fright'],regions['fleft'])

    # Publishing Angular and Linear Velocities
    pub.publish(velocity_msg)


# Distance To Line - A function to calculate the distance to the imaginary line (line between the initial point and the goal point)
def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the imaginary line
    p1 = initial
    p2 = Point()
    p2.x = final_x
    p2.y = final_y

    # Calculation of the distance to the line
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x)
                      * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance

# Path Tracing - A function to make the bot follow a given path using the generated waypoints
def path_tracing(pangle):

    while not rospy.is_shutdown():
        for t in [pangle]:
            # Generating the next waypoint
            [goal.x, goal.y] = waypoint(t)

            # Calculating difference in current and desired points
            inc_x = goal.x - x
            inc_y = goal.y - y

            # Calculating angle towards the goal
            angle_to_goal = round(math.atan2(inc_y, inc_x), 1)

            # P only Controller
            diff = angle_to_goal-theta
            p = 1.1
            error = p*diff

            # Tolerance
            theta_prec = math.pi/90
            dist_prec = 0.3

            # Calculating distance between current and desired points
            err_pos = math.sqrt(pow(goal.y - y, 2) + pow(goal.x - x, 2))

            # Turning Anti-clockwise
            if (error) > theta_prec:
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = 0.7 + error

            # Turning Clockwise
            elif (error) < -theta_prec:
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = -0.7 + error

            # Moving Forward
            elif (err_pos > dist_prec):
                velocity_msg.linear.x = 0.7
                velocity_msg.angular.z = 0.0

            # Checking and Generating Waypoints
            elif (err_pos < dist_prec):
                # Before 2pi [Path Tracing]
                if (pangle < 360):
                    path_tracing(pangle+45)

                # After 2pi [Obstacle Avoidance]
                else:
                    # Setting Initial Point
                    initial.x = x
                    initial.y = y
                    d = 1.85

                    while True:
                        # Calculating distance to the imaginary line
                        distance_position_to_line = distance_to_line(position_)

                        # Condition for Go To Goal
                        if distance_position_to_line < 0.3 and (regions['front'] > d and regions['fleft'] > d):
                            go_to_goal(final_x, final_y)

                        # Condition for Obstacle Avoider
                        elif (regions['front'] < d and regions['fleft'] < d and regions['fright'] < d) or distance_position_to_line > 0.1:
                            obstacle(regions)

        # Publishing Angular and Linear Velocities
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
