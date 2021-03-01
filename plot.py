#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import time

def readInput (filename):
    f = open (filename, 'r')
    lines = f.readlines()
    f.close ()
    x_ = []
    y_ = []

    for line in lines:
        (x, y) = (int (line.split(' ')[0]), int (line.split(' ')[1]))
        x_.append (x)
        y_.append (y)

    return (x_, y_)


(x0, y0) = readInput ('waypoints_0')
(x1, y1) = readInput ('waypoints_1')
(x2, y2) = readInput ('waypoints_2')
(x3, y3) = readInput ('waypoints_3')

# (x0, y0) = readInput ('robot0.plan')
# (x1, y1) = readInput ('robot1.plan')
# (x2, y2) = readInput ('robot2.plan')
# (x3, y3) = readInput ('robot3.plan')

# obsx = [4, 4, 5, 5]
# obsy = [3, 4, 3, 4]

obsx = [2, 3, 1, 3, 1, 2]
obsy = [0, 0, 2, 2, 4, 4]

initialx = [x0[0], x1[0], x2[0], x3[0]]
initialy = [y0[0], y1[0], y2[0], y3[0]]

plt.scatter (initialx, initialy, c=['r','b','g','y'])
# plt.scatter (obsx, obsy, c=['k','k','k','k','k','k'])
plt.ylim ((-1, 10))
plt.xlim ((-1, 10))
plt.pause (1)

for i in range (0, len (x0), 1):
    plt.plot (x0[i:i+2], y0[i:i+2], 'r--')
    plt.plot (x1[i:i+2], y1[i:i+2], 'b--')
    plt.plot (x2[i:i+2], y2[i:i+2], 'g--')
    plt.plot (x3[i:i+2], y3[i:i+2], 'y--')
    plt.pause (0.5)

plt.show ()
