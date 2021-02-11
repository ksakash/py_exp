#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped

current_state = State ()
curr_pose = PoseStamped ()

def state_cb (data):
    current_state = data

def pose_cb (data):
    curr_pose = data

rospy.init_node ('offboard_control')

state_sub = rospy.Subscriber("mavros/state", 10, state_cb)
pose_sub = rospy.Subscriber("mavros/local_position/pose", 10, pose_cb)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", 10)

rospy.wait_for_service ('mavros/cmd/arming')
rospy.wait_for_service ('mavros/set_mode')

arming_client = rospy.ServiceProxy ("mavros/cmd/arming", CommandBool)
set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)

rate = rospy.Rate (20)

while(not rospy.is_shutdown () and  not current_state.connected):
    rospy.spinOnce()
    rate.sleep()

pose = PoseStamped ()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

for i in range (100):
    local_pos_pub.publish (pose)
    rospy.spinOnce ()
    rate.sleep ()

last_request = rospy.Time.now ()

while not rospy.is_shutdown ():
    if ((current_state.mode != 'OFFBOARD') and \
        ((rospy.Time.now () - last_request) > rospy.Duration (5.0))):
        res = set_mode_client (custom_mode="OFFBOARD")
        if (res.mode_sent):
            print ("Offboard enabled")
        last_request = rospy.Time.now ()
    elif ((not current_state.armed) and \
        ((rospy.Time.now () - last_request) > rospy.Duration (5.0))):
        res = arming_client (value=True)
        if (res.success):
            print ("Vehicle armed")
        last_request = rospy.Time.now ()
    local_pos_pub.publish (pose)
    rospy.spinOnce ()
    rospy.sleep ()
