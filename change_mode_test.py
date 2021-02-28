#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math
import time

import sys

current_state = State ()
def state_cb (data):
    global current_state
    current_state = data

rospy.init_node ('change_mode_test')
rate = rospy.Rate (20)

state_sub = rospy.Subscriber("mavros/state", State, state_cb)

print ("CHANGE_MODE TEST INTIALIZING...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print ("waiting for the connection with FCU")
    rate.sleep()

rospy.wait_for_service ('mavros/set_mode')
print ("mavros/set_mode service is active...")

mode = "GUIDED"

set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)
mode_res = set_mode_client (custom_mode=mode)

if mode_res.mode_sent:
    print (mode, "mode sent", mode_res.mode_sent)
else:
    print ("ERROR in setting", mode, "mode!")
    sys.exit (-1)

mode = "STABILIZE"
mode_res = set_mode_client (custom_mode=mode)

if mode_res.mode_sent:
    print (mode, "mode sent", mode_res.mode_sent)
else:
    print ("ERROR in setting", mode, "mode!")
    sys.exit (-1)

print ("change mode tested successfully !!")
