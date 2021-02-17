#!/usr/bin/env python2.7

import geometry_msgs.msg
import tf2_ros
from geometry_msgs.msg import Point
from find_object_2d.msg import ObjectsStamped
from apriltag_ros.msg import AprilTagDetection
from apriltag_ros.msg import AprilTagDetectionArray
from object_msgs.msg import ObjectPose
import rospy
import cv2
import matplotlib as plt
import time
import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
    uuid, ['/home/akhiljayan29aj/catkin_ws1/src/object_recognition/launch/img_saver.launch'])
launch.start()

time.sleep(0.8)

launch.shutdown()

tf = tf2_ros


def callback(data):
    image = cv2.imread('/home/akhiljayan29aj/Desktop/d.jpg')
    thickness = 2
    fontScale = 1
    tfbuffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tfbuffer)

    # Blue color in BGR
    color = (0, 0, 255)
    # Line thickness of 2 px
    thickness = 2
    for i in range(0, len(data.objects.data), 12):
        try:
            d = data.objects.data
            l = len(d)
            # ===== Meeting Room ======

            print("Meeting Room")
            if l > 0:
                if d[i] == 7:
                    image = cv2.putText(image, "Glue", (int(d[9+i]), int(
                        d[i+10])), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[9+i]), int(d[i+10])), (int(
                        d[i+1]+d[i+9]), int(d[i+2]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('Glue Identified')
            if l > 13:
                if d[i] == 8:
                    image = cv2.putText(image, "Battery", (int(d[9+i]), int(
                        d[i+10])), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[9+i]), int(d[i+10])), (int(
                        d[i+1]+d[i+9]), int(d[i+2]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('Battery Identified')
            if l > 25:
                if d[i] == 9 or d[i] == 23 or d[i] == 24 or d[i] == 25:
                    image = cv2.putText(image, "Adhesive", (int(d[9+i]), int(
                        d[i+10])), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[9+i]), int(d[i+10])), (int(
                        d[i+1]+d[i+9]), int(d[i+2]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('Adhesive Identified')

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        k = cv2.waitKey(40) & 0xFF
        if k == 2:
            break
        cv2.imshow('ii', image)
        continue
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    pub = rospy.Publisher(
        'tag_detections', AprilTagDetectionArray, queue_size=10)
    rospy.init_node('objects_to_tags', anonymous=True)
    rospy.Subscriber("objectsStamped", ObjectsStamped, callback)

    rospy.spin()
