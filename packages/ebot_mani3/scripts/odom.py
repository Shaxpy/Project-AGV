#!/usr/bin/env python
import cv2
import rospy
from object_msgs.msg import ObjectPose
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import Point
import tf2_ros
import geometry_msgs.msg
objFramePrefix_ = "object"
distanceMax_ = 0.0
tf = tf2_ros


def callback(data):
    image = cv2.imread('/home/akhiljayan29aj/catkin_ws1/src/pic/d.jpg')
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
            image = cv2.rectangle(image, (int(d[9]), int(d[10])), (int(
                d[1]+d[9]), int(d[2]+d[10])),   (0, 255, 0), thickness)
            image = cv2.rectangle(image, (int(d[21]), int(d[22])), (int(
                d[13]+d[21]), int(d[14]+d[22])),   (0, 255, 0), thickness)
            # image = cv2.rectangle(image, (int(d[33]), int(d[34])), (int(
            #     d[25]+d[33]), int(d[26]+d[34])),   (0, 255, 0), thickness)
            if d[0] == 51:
                image = cv2.putText(image, "coke", (int(d[9]), int(
                    d[10])-10), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
            if d[12] == 52:
                image = cv2.putText(image, "water glass", (int(d[21]), int(
                    d[22])-10), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
            # if d[24] == 45:
            #     image = cv2.putText(image, "Glue", (int(d[33]), int(
            #         d[34])-10), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)

            # print d[12]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    cv2.imshow('ii', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    pub = rospy.Publisher(
        'tag_detections', AprilTagDetectionArray, queue_size=10)
    rospy.init_node('objects_to_tags', anonymous=True)
    rospy.Subscriber("objectsStamped", ObjectsStamped, callback)

    rospy.spin()
