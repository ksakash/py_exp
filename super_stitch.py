#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose

import sys
sys.path.append ('../drone_image_stitching')

class ImageStitch (object):
    def __init__ (self):
        self.pose = None
        self.curr_img = None
        self.result = None
        self.bridge = CvBridge ()

    def pose_cb (self, data):
        self.pose = data.pose

    def image_sb (self, data):
        try:
            curr_img = self.bridge.imgmsg_to_cv2 (data, "bgr8")
        except CvBridgeError as e:
            print (e)

        self.combine (curr_img)

    def combine (self, data):
        pass

if __name__ == '__main__':
    rospy.init_node ('super_stitch')

    # subscribe to the data coming from the quadcopter
    pose_sub = rospy.Subscriber ("pose topic name", 10, pose_cb)
    image_sub = rospy.Subscriber ("image topic name", 10, image_cb)

    # to talk to the controller
    pose_pub = rospy.Publisher ("controller pose topic name", Pose, queue_size=10, latch=True)
    rospy.spin ()
