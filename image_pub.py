#!/usr/bin/env python3

import os
import roslib
import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node ('image_capture', anonymous=True)
    image_pub = rospy.Publisher ("/image_stitching", Image, queue_size=5, latch=True)
    pose_pub = rospy.Publisher ("/image_pose", Float32MultiArray, queue_size=5, latch=True)
    
    dirname = 'images'
    filename = 'imagesData.txt'
    bridge = CvBridge ()

    data_matrix = np.genfromtxt (filename, delimiter=",", usecols=range(1, 7), dtype=float)
    image_name_matrix = np.genfromtxt (filename, delimiter=",", usecols=[0], dtype=str)    
    count = 0

    for image_name in image_name_matrix:

        image_path = dirname + '/' + image_name
        img = cv2.imread (image_path)
        image_message = bridge.cv2_to_imgmsg (img, encoding="passthrough")
        image_pub.publish (image_message)

        msg = Float32MultiArray ()
        msg.data = list (data_matrix[count])
        pose_pub.publish (msg)

        count += 1

        time.sleep (4.0)

