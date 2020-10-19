#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

'''
Making a Class(TaskZero) with objects to perform functions like:
- Publishing and Subscribing
- Making the turtle form a circle and stopping
'''


class TaskZero():
    def __init__(self):
        # Publisher
        rospy.init_node('node_turtle_revolve', anonymous=True)
        self.vel_pub = rospy.Publisher(
            '/turtle1/cmd_vel', Twist, queue_size=10)
        # Subscriber
        self.pose_sub = rospy.Subscriber(
            '/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    # Subscriber Callback function

    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def circle(self):
        radius = 1.000
        # Get the Pose
        goal_pose = Pose()
        vel_msg = Twist()
        self.rate.sleep()

        # Time at beginning
        initial_t = float(rospy.Time.now().to_sec())
        dist = 0.000
        # Circumference
        circum = 2.0*3.141592653*radius
        # Loop for Distance travelled
        while circum >= dist:
            # Linear Velocity
            vel_msg.linear.x = circum
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular Velocity
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 2.0*3.141592653
            # Time at end
            final_t = float(rospy.Time.now().to_sec())
            dist = vel_msg.linear.x * (final_t-initial_t)
            print " Moving in a circle "
            print str(dist)

        # Publish velocity
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()
        # Brakes
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        print "Goal Reached"

# Main Function initialised


if __name__ == '__main__':
    try:
        make = TaskZero()
        make.circle()
    except rospy.ROSInterruptException:
        pass

