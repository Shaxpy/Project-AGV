#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def main():
    rospy.init_node('talker', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 5
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 1
	
	#Move Robot in circle
    while not rospy.is_shutdown():
            pub.publish(vel_msg)
    vel_msg.linear.x = 0	
    vel_msg.linear.z = 0
    velocity_publisher.publish(vel_msg)
# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
