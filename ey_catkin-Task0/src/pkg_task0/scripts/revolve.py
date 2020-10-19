#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist

def revolve_turtle(l_vel,a_vel):
    rospy.init_node('revolve', anonymous=True)
    vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    speed=rospy.Rate(5)
    vel_msg = Twist()
	
	#Move Robot in circle
    while not rospy.is_shutdown():
 	vel_msg.linear.x = l_vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = a_vel
     	rospy.loginfo("Moving in circle :",l_vel,a_vel)
	vel.publish(vel_msg)
	speed.sleep()
# Python Main
if __name__ == '__main__':
    try:
        revolve_turtle(5.0,2.5)
    except rospy.ROSInterruptException:
        pass
