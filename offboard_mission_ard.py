#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math
import time

rospy.init_node ('offboard_control_ardupilot')
rate = rospy.Rate (20)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10, latch=True)

current_state = State ()
curr_pose = PoseStamped ()

def state_cb (data):
    global current_state
    current_state = data

def pose_cb (data):
    global curr_pose
    curr_pose = data

time_limit = 3
hit_frequency = 40
target_margin = 0.2

def within_range (target, curr):
    diffx = abs (target.pose.position.x - curr.pose.position.x)
    diffy = abs (target.pose.position.y - curr.pose.position.y)
    diffz = abs (target.pose.position.z - curr.pose.position.z)
    global target_margin
    m = target_margin

    if diffx <= m and diffy <= m and diffz <= m:
        return True
    else:
        return False

def reached_target (target):
    global time_limit, hit_frequency
    then = time.time ()
    count = 0
    rate = rospy.Rate (100)
    while ((time.time () - then) < time_limit and not rospy.is_shutdown ()):
        if (within_range (target, curr_pose)):
            count += 1
        if (count >= hit_frequency):
            return True
        rate.sleep()
    if (count >= hit_frequency):
        return True
    else:
        return False

def target_handler (plan):
    rate = rospy.Rate (20)
    if len (plan) == 0:
        return
    next_target = plan[0]
    while (len (plan) > 0 and not rospy.is_shutdown ()):
        if (reached_target (next_target)):
            print ("Achieved:", next_target.pose.position.x, next_target.pose.position.y, next_target.pose.position.z)
            plan = plan[1:]
            if (len (plan) > 0):
                next_target = plan[0]
        local_pos_pub.publish (next_target)
        rate.sleep ()

def process_input (filename):
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    plan = []

    for line in lines:
        (x, y, z) = (int (line.split(' ')[0]), int (line.split(' ')[1]), int (line.split(' ')[2]))
        wp = PoseStamped ()
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z

        plan.append (wp)

    return plan

state_sub = rospy.Subscriber("mavros/state", State, state_cb)
pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)

print ("INTIALIZING...")
time.sleep (1)

while(not rospy.is_shutdown () and not current_state.connected):
    print ("waiting for the state to be connected")
    rate.sleep()

rospy.wait_for_service ('mavros/set_mode')
print ("mavros/set_mode service is active...")

set_mode_client = rospy.ServiceProxy ("mavros/set_mode", SetMode)
mode_res = set_mode_client (custom_mode="GUIDED")

if mode_res.mode_sent:
    print ("guided mode sent", mode_res.mode_sent)
else:
    print ("failed guided mode!")

while (current_state.mode != 'GUIDED' and not rospy.is_shutdown ()):
    print ("current state mode:", current_state.mode)
    time.sleep (0.01)

print ("reading input..")
filename = "waypoints_2"
plan = process_input (filename)

rospy.wait_for_service ('mavros/cmd/arming')
print ("mavros/cmd/arming service is active...")

arming_client = rospy.ServiceProxy ("mavros/cmd/arming", CommandBool)

arm_res = arming_client (value=True)
if arm_res.success:
    print ("ARM sent", arm_res.success)
else:
    print ("Failed Arming!")

takeoff_client = rospy.ServiceProxy ("mavros/cmd/takeoff", CommandTOL)
takeoff_res = takeoff_client (altitude=1.5)

if takeoff_res.success:
    print ("takeoff sent", takeoff_res.success)
else:
    print ("failed takeoff!")

print ("starting the mission..")
target_handler (plan)
print ("completed the mission!!")

rtl = False

if rtl:
    print ("RTL mode ...")
    mode_res = set_mode_client (custom_mode="RTL")

    if mode_res.mode_sent:
        print ("RTL mode sent", mode_res.mode_sent)
    else:
        print ("failed RTL mode!")
else:
    land_client = rospy.ServiceProxy ("mavros/cmd/land", CommandTOL)
    land_res = land_client ()

    if land_res.success:
        print ("land sent", land_res.success)
    else:
        print ("landing failed!")
        rospy.shutdown ()

while not rospy.is_shutdown ():
    rate.sleep ()
