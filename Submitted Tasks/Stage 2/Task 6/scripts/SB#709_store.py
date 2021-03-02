#!/usr/bin/env python2.7

# Python Script for generating bounding boxes in STORE ROOM
# REFERENCES
# http://wiki.ros.org/apriltag_ros

# Importing libraries
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

# universally unique identifier
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# launching launch file for saving an image
launch = roslaunch.parent.ROSLaunchParent(
    uuid, ['/home/akhiljayan29aj/catkin_ws1/src/object_recognition/launch/img_saver.launch'])
launch.start()

time.sleep(0.8)

launch.shutdown()

tf = tf2_ros

# Callback function for object stamped node


def callback(data):

    # Reading the saved image
    image = cv2.imread('/home/akhiljayan29aj/Desktop/d.jpg')

    thickness = 2
    fontScale = 1
    tfbuffer = tf2_ros.Buffer()

    # ID's for respective objects
    fpga = [10, 11, 22]
    adh = [12,  13, 30, 221]
    glue = [14, 15]
    battery = [16, 17, 32, 219, 220, 222]
    wheels = [18, 19, 27, 29, 33]
    eyfi = [20, 21, 31]

    # Receiving TF2 Transformations
    listener = tf2_ros.TransformListener(tfbuffer)
    # Blue color in BGR
    color = (0, 0, 255)
    # Line thickness of 2 px
    thickness = 2

    for i in range(0, len(data.objects.data), 12):
        try:
            # Tuple containing the details of the objects identified
            d = data.objects.data

            # l holds the length of the tuple published on the topic ObjectStamped
            l = len(d)
            # computer vision for bounding boxes
            # Determines the number and the name of objects identified based on the value of l
            if l > 0:
                if d[i] in fpga:
                    # Generating bounding boxes with label
                    image = cv2.putText(image, "FPGA Board", (int(d[i+9]), int(
                        d[10+i])-5), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[i+9]), int(d[i+10])), (int(
                        d[1+i]+d[9+i]), int(d[2+i]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('FPGA Board Identified')

                if d[i] in adh:
                    image = cv2.putText(image, "Adhesive", (int(d[i+9]), int(
                        d[10+i])-5), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[i+9]), int(d[i+10])), (int(
                        d[1+i]+d[9+i]), int(d[2+i]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('Adhesive Identified')

                if d[i] in glue:
                    image = cv2.putText(image, "Glue", (int(d[9+i]), int(
                        d[10+i]-5)), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[i+9]), int(d[i+10])), (int(
                        d[1+i]+d[9+i]), int(d[2+i]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('Glue Identified')

                if d[i] in battery:
                    image = cv2.putText(image, "Battery", (int(d[9+i]), int(
                        d[10+i])-5), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[i+9]), int(d[i+10])), (int(
                        d[1+i]+d[9+i]), int(d[2+i]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('Battery Identified')

                if d[i] in wheels:
                    image = cv2.putText(image, "Robot wheels", (int(d[9+i]), int(
                        d[10+i])-5), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[i+9]), int(d[i+10])), (int(
                        d[1+i]+d[9+i]), int(d[2+i]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('Robot Wheels Identified')

                if d[i] in eyfi:
                    image = cv2.putText(image, "eYFI Board", (int(d[9+i]), int(
                        d[10+i])-5), cv2.FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv2.LINE_AA)
                    image = cv2.rectangle(image, (int(d[i+9]), int(d[i+10])), (int(
                        d[1+i]+d[9+i]), int(d[2+i]+d[i+10])),   (0, 255, 0), thickness)
                    rospy.loginfo('EYFI Board Identified')

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # updating image in every 40 milliseconds
        k = cv2.waitKey(40) & 0xFF
        if k == 2:
            break
        # Showing the identified objects
        cv2.imshow('ii', image)
        continue


# Main function
if __name__ == '__main__':

    # Create a handle to publish messages to tag_detections topic
    pub = rospy.Publisher(
        'tag_detections', AprilTagDetectionArray, queue_size=10)

    # Initializes the ROS node for the process
    rospy.init_node('objects_to_tags', anonymous=True)

    # Subcribes to the topic ObjectStamped for obtaining the coordinated for making the bounding boxes
    rospy.Subscriber("objectsStamped", ObjectsStamped, callback)

    rospy.spin()
