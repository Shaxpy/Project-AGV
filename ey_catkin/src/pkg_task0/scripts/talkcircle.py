#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def talker():
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.init_node('node_turtle_revolve', anonymous=True)
    vel_msg = Twist()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        vel_msg.linear.x = 3.0
        vel_msg.angular.z = 1.5
        rospy.loginfo("Moving in circle:")
        rospy.loginfo(vel_msg)
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__' :
    try:
        talker()
    except rospy.ROSInterrptException:
        pass


            









