#!/usr/bin/env python3

import airsim
import cv2
import numpy as np
import os
import pprint
import setup_path
import tempfile
import threading

robots = [0, 1]
clients = []

intial_pos = [(0, 0), (1, 0), (0, 1), (1, 1)]

for id in robots:
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    clients.append (client)

for id in robots:
    vehicle_name = "Drone" + str (id)
    clients[id].enableApiControl (True, vehicle_name)
    clients[id].armDisarm (True, vehicle_name)

airsim.wait_key('Press any key to takeoff')

def readInput (filename):
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    arr = []

    for line in lines:
        (x, y) = (int (line.split(' ')[0]), int (line.split(' ')[1]))
        arr.append ((x, y))

    return arr

h = 25
image_width = 1920
image_height = 1080
unit_y = 0.2 * h
unit_x = (image_height/image_width) * unit_y

def perform_course ():
    waypoints = []
    for id in robots:
        filename = "waypoints_" + str (id)
        waypoint = readInput (filename)
        waypoints.append (waypoint)

    num_points = len (waypoints[0])
    max_speed = 2
    print ("no. of points:", num_points)

    for i in range (num_points):
        print ("point:", i)
        threads = []
        for id in robots:
            vehicle_name = "Drone" + str (id)
            x = (waypoints[id][i][1] - intial_pos[id][1]) * unit_x
            y = (waypoints[id][i][0] - intial_pos[id][0]) * unit_y
            z = -h
            print ("robot id:", id, "x:", x, "y:", y, "z:", z)
            threads.append (clients[id].moveToPositionAsync(x, y, z, \
                            max_speed, vehicle_name=vehicle_name))

        for th in threads:
            th.join ()

perform_course ()

airsim.wait_key('Press any key to reset to original state')

for id in robots:
    vehicle_name = "Drone" + str (id)
    clients[id].armDisarm(False, vehicle_name)
    clients[id].reset()
    clients[id].enableApiControl(False, vehicle_name)
