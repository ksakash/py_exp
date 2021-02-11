#!/usr/bin/env python3

from collections import deque
import rospy
import time

queue_size = 100

imu_de = deque ()
gps_de = deque ()

imu_data = None
gps_data = None

def imu_callback (data):
    imu_de.append (data)
    if (len (imu_de) > queue_size):
        global imu_data
        imu_data = imu_de.popleft ()

def gps_callback (data):
    gps_de.append (data)
    if (len (gps_de) > queue_size):
        global gps_data
        gps_data = gps_de.popleft ()

rospy.init_node ('geotagging_images')
imu_sub = rospy.Subscriber ('/imu_topic_name', 10, imu_callback)
gps_sub = rospy.Subscriber ('/gps_topic_name', 10, gps_callback)

rate = rospy.Rate (10)

while not rospy.is_shutdown ():
    if imu_data is not None:
        pass
    if gps_data is not None:
        pass
    rate.sleep ()
    rospy.spinOnce ()
