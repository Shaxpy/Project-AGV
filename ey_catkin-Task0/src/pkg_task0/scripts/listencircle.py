#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def pose_callback(msg):
    print(msg.theta)

def listener():
    rospy.init_node('listenercircle', anonymous=True)

    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptExecution:
        pass
    



