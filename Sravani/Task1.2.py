#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math

x = 0
y = 0
theta = 0
final_x = 12.5
final_y = 0
position_ = Point()
initial = Point()


def main():
    global pub, goal, rate
    global velocity_msg
    velocity_msg = Twist()

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    goal = Point()
    state = 0
    rospy.init_node('reading_laser')
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    control_loop(1)
    rospy.spin()
    pub.publish(velocity_msg)


def Waypoint(t):
    x = ((math.pi*t)/180)
    y = (2*(math.sin(x))*(math.sin(x/2)))
    return [round(x, 1), round(y, 1)]


def odom_callback(msg):
    global pose
    global x
    global y
    global theta
    global position_

    # position
    position_ = msg.pose.pose.position
    x = round(msg.pose.pose.position.x, 1)
    y = round(msg.pose.pose.position.y, 1)
    rot_q = msg.pose.pose.orientation
    pose = (roll, pitch, th) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    theta = round(th, 1)


def laser_callback(msg):
    global regions
    regions = {
        'right':  min(min(msg.ranges[0:144]), 20),
        'fright': min(min(msg.ranges[145:289]), 20),
        'front':  min(min(msg.ranges[290:434]), 20),
        'fleft':  min(min(msg.ranges[435:579]), 20),
        'left':   min(min(msg.ranges[580:720]), 20),
    }
    rospy.loginfo(regions)


def go_to_waypoint(final_x, final_y):
    inc_x = final_x - x
    inc_y = final_y - y
    angle_to_goal = round(math.atan2(inc_y, inc_x), 1)
    diff = angle_to_goal-theta
    p = 1.1
    error = p*diff
    theta_prec = math.pi/90
    dist_prec = 0.3
    err_pos = math.sqrt(pow(final_y - y, 2) + pow(final_x - x, 2))
    print "error=="+str(error)
    print "theta=="+str(theta)
    print "angle=="+str(angle_to_goal)

    if (error) > theta_prec:

        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.9 + error
    # turn clockwise
    elif (error) < -theta_prec:

        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = -0.9 + error

    # move forward
    elif (err_pos > dist_prec):

        velocity_msg.linear.x = 0.9
        velocity_msg.angular.z = 0.0
    # stop
    elif (err_pos < 0.2):

        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
    pub.publish(velocity_msg)


def obstacle(regions):
    print("Obstacle loop")
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    state_description = ''
    d = 1.5
    if regions['front'] > d and regions['fleft'] > d and regions['left'] > d and regions['right'] > d:
        state_description = ' Straight'
        velocity_msg.linear.x = 0.7
        velocity_msg.angular.z = 0
    elif regions['front'] < d and regions['left'] > d and regions['right'] > d and regions['fleft'] < d:
        state_description = ' Left'
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0.55
    elif regions['front'] < d and regions['left'] > d and regions['fright'] < d and regions['fleft'] > d:
        state_description = ' more Left'
        velocity_msg.linear.x = 0.12
        velocity_msg.angular.z = 0.57
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = ' left straight'
        velocity_msg.linear.x = 0.8
        velocity_msg.angular.z = 0
    elif regions['front'] > d and regions['left'] > d and regions['fleft'] > d and regions['right'] < d:
        state_description = ' Rgt straight'
        velocity_msg.linear.x = 0.3
        velocity_msg.angular.z = -0.68
    #elif regions['front'] > d and regions['fleft'] > d and regions['left'] > d and regions['right'] < d:
    #     state_description = 'Goal'
    #     while True:
    #         goal(12.5, -1)


    # rospy.loginfo(regions)
    # rospy.loginfo(state_description)
    # msg.linear.x = velocity_msg.linear.x
    # msg.angular.z = angular_z
    pub.publish(velocity_msg)


def change_state(state):
    global state_
    state_ = state
    if state_ == 0:
        go_to_waypoint(final_x, final_y)
    elif state_ == 1:
        obstacle(regions)


def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    p1 = initial
    p2 = Point()
    p2.x = final_x
    p2.y = final_y
    # here goes the equation
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x)
                      * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq
    print(p1)
    print(p2)
    return distance


def control_loop(pangle):
    global regions

    while not rospy.is_shutdown():
        # Your algorithm to complete the obstacle course

        for t in [pangle]:
            [goal.x, goal.y] = Waypoint(t)
            inc_x = goal.x - x
            inc_y = goal.y - y
            angle_to_goal = round(math.atan2(inc_y, inc_x), 1)
            print("inside loop", pangle)
            print("x = {}".format(x))
            print("y = {}".format(y))
            print("goal.x = {}".format(goal.x))
            print("goal.y = {}".format(goal.y))
            diff = angle_to_goal-theta
            p = 1.1
            error = p*diff
            theta_prec = math.pi/90
            dist_prec = 0.3
            err_pos = math.sqrt(pow(goal.y - y, 2) + pow(goal.x - x, 2))

            # print("theta = {}".format(theta))
            # print("angle_to_goal = {}".format(angle_to_goal))

            # turn anticlockwise
            if (error) > theta_prec:

                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = 0.7 + error
            # turn clockwise
            elif (error) < -theta_prec:

                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = -0.7 + error
                # move forward
            elif (err_pos > dist_prec):
                # else:
                velocity_msg.linear.x = 0.7
                velocity_msg.angular.z = 0.0
            # checking and generating waypoints
            elif (err_pos < dist_prec):
                # elif(x == goal.x):
                if (pangle < 360):
                    control_loop(pangle+45)
                # stop
                else:
                    initial.x = x
                    initial.y = y
                    d = 1.85
                    # velocity_msg.linear.x = 0
                    # velocity_msg.angular.z = 0
                    
                    
                    print("inside Control wala while loop")
                    while True:
                        distance_position_to_line = distance_to_line(position_)
                        print str(distance_position_to_line)
                        #if (regions['front'] > d and regions['fleft'] > d and regions['fright'] > d)or (regions['front'] > d and regions['left'] > d and regions['right'] < d) or (regions['front'] > d and regions['left'] < d and regions['right'] > d) and distance_position_to_line < 0.1:
                        if distance_position_to_line < 1.0 and (regions['front'] > d and regions['fleft'] > d ) :
                        #or (regions['left'] > d and regions['fright'] < d/2 and regions['right'] < d/2):
                            # WAYPOINT
                            change_state(0)
                            print("state changed to 0")
                            print str(distance_position_to_line)

                        elif (regions['front'] < d and regions['fleft'] < d and regions['fright'] < d) or distance_position_to_line > 0.1 :
                            # OBSTACLE AVOIDER
                            change_state(1)
                            print str(distance_position_to_line)
                            print("state changed to 1")


        pub.publish(velocity_msg)
        print("#############################################################")
        print("Controller message pushed at {}".format(rospy.get_time()))
        print("#############################################################")
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
