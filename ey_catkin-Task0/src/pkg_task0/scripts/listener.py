#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def main():
    rospy.init_node('listener', anonymous=True)   
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    rospy.spin()




def pose_callback(msg):
    print(msg.theta)

# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

