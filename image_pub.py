#!/usr/bin/env python3

import os
import roslib
import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node ('image_capture', anonymous=True)
    image_pub = rospy.Publisher ("/image_stitching", Image, queue_size=5, latch=True)
    pose_pub = rospy.Publisher ("/image_pose", PoseStamped, queue_size=5, latch=True)

    dirname = 'airsim_images'
    filename = 'airsimImage.txt'
    bridge = CvBridge ()

    data_matrix = np.genfromtxt (filename, delimiter=",", usecols=range(1, 7), dtype=float)
    image_name_matrix = np.genfromtxt (filename, delimiter=",", usecols=[0], dtype=str)
    count = 0

    # for image_name in image_name_matrix:
    while not rospy.is_shutdown () and count < len (data_matrix):
        msg = PoseStamped ()
        msg.pose.position.x = data_matrix[count][0]
        msg.pose.position.y = data_matrix[count][1]
        msg.pose.position.z = data_matrix[count][2]
        pose_pub.publish (msg)

        image_name = image_name_matrix[count]
        image_path = dirname + '/' + image_name
        img = cv2.imread (image_path)
        image_message = bridge.cv2_to_imgmsg (img, encoding="bgr8")
        image_pub.publish (image_message)

        count += 1
        time.sleep (4.0)
