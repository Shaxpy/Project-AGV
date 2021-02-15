#!/usr/bin/env python
import roslib
import rospy
import math
import tf2_ros
import PyKDL
import geometry_msgs.msg
from math import pi


if __name__ == '__main__':
    rospy.init_node('tf_echo')

    tfbuffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tfbuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfbuffer.lookup_transform(
                'odom', 'ebot_base',  rospy.Time())
            # trans = trans.transform.translation.x
            print trans
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print "Fail", e

        rate.sleep()
