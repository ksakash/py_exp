#!/usr/bin/env python3

from z3 import *
import numpy as np

def fill_obstacle_tocover (x_, y_, obstacles, visible, l, map):
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    x_l = int (max (0, x_-l))
    x_h = int (min (dimension_x, x_+l+1))
    y_l = int (max (0, y_-l))
    y_h = int (min (dimension_y, y_+l+1))
    for x in range (x_l, x_h):
        for y in range (y_l, y_h):
            if (map[y][x] == 0.0) and (x,y) not in obstacles:
                obstacles.append ((x,y))
            elif (map[y][x] == 0.5) and (x,y) not in visible:
                visible.append ((x,y))

def safe_space (x_, y_, l, map):
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    x_l = int (max (0, x_-l))
    x_h = int (min (dimension_x, x_+l+1))
    y_l = int (max (0, y_-l))
    y_h = int (min (dimension_y, y_+l+1))
    arr = []
    for x in range (x_l, x_h):
        for y in range (y_l, y_h):
            if map[y][x] == 0.5 or map[y][x] == 1.0:
                arr.append ((x,y))
    return arr

map = np.array ([[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.0,0.0,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.0,0.0,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5]])

local_map = np.zeros_like (map)

print (map.shape)

num_covered = 0
total = map.shape[0] * map.shape[1]
R = 4
local_range = 1

start_x = np.empty ((R,))
start_y = np.empty ((R,))

start_x[0] = 0
start_x[1] = 0
start_x[2] = 2
start_x[3] = 2

start_y[0] = 0
start_y[1] = 2
start_y[2] = 0
start_y[3] = 2

dimension_x = map.shape[1]
dimension_y = map.shape[0]

obstacles = []
visible = []
covered = []

for r in range (R):
    fill_obstacle_tocover (start_x[r], start_y[r], obstacles, visible, local_range, map)

k = 0
files = []

for r in range (R):
    filename = 'waypoints_' + str (r)
    f = open (filename, 'w+')
    files.append (f)

motion_w = 1 # cost of taking each step
old_w = 1 # reward of visiting old grid first
visible_w = 1 # reward of covering visible grids which are near
not_covered_w = 0 # cost of covering already covered grid

while num_covered < total:
    for r in range (R):
        print ("robot " + str (r) + ":", start_x[r], start_y[r])

    num_visible = len (visible)

    T = 4
    s = Optimize ()
    total_c = Int ('total_c')
    total_re = Int ('total_re')

    X = [[Int("x_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    Y = [[Int("y_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    P = [[Int("p_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    S = [[Int("s_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    NC = [[Int("NC_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

    Re = [Int("re_%s" % (j)) for j in range(num_visible)]

    C = [[Int("c_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

    s.add (total_c == Sum (Re + S[0] + S[1] + S[2] + S[3] + C[0] + C[1] + C[2] + C[3]))

    for r in range (R):
        s.add (And (X[r][0] == start_x[r], Y[r][0] == start_y[r]))
        s.add (Or (X[r][T-1] != start_x[r], Y[r][T-1] != start_y[r]))

    print ("obstacles:", obstacles)
    print ("visible:", visible)

    for i in range (num_visible):
        s.add (Or (Re[i] <= 2 * num_visible, Re[i] >= 0))

    # obstacle avoidance
    for r in range (R):
        for t in range (T):
            for obst in obstacles:
                s.add (Or (X[r][t] != obst[0], Y[r][t] != obst[1]))

            # stay within bounds
            s.add (And (X[r][t] < dimension_x, X[r][t] >= 0))
            s.add (And (Y[r][t] < dimension_y, Y[r][t] >= 0))

            s.add (And (P[r][t] < 5, P[r][t] >= 0))
            s.add (Or (C[r][t] == 3 * motion_w, C[r][t] == 5 * motion_w))
            s.add (And (S[r][t] >= 0, S[r][t] <= 32))
            s.add (Or (NC[r][t] == not_covered_w, NC[r][t] == 1000))

    # motion primitives
    for r in range (R):
        for t in range (T-1):
            s.add(Implies(P[r][t] == 0, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t], C[r][t] == 3 * motion_w))) # same
            s.add(Implies(P[r][t] == 1, And(X[r][t+1] == X[r][t]+1, Y[r][t+1] == Y[r][t], C[r][t] == 5 * motion_w))) # right
            s.add(Implies(P[r][t] == 2, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]+1, C[r][t] == 5 * motion_w))) # up
            s.add(Implies(P[r][t] == 3, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]-1, C[r][t] == 5 * motion_w))) # down
            s.add(Implies(P[r][t] == 4, And(X[r][t+1] == X[r][t]-1, Y[r][t+1] == Y[r][t], C[r][t] == 5 * motion_w))) # left

    # collision avoidance
    for t in range(0, T):
        s.add(Or(X[0][t] != X[1][t], Y[0][t] != Y[1][t]))
        s.add(Or(X[0][t] != X[2][t], Y[0][t] != Y[2][t]))
        s.add(Or(X[0][t] != X[3][t], Y[0][t] != Y[3][t]))
        s.add(Or(X[1][t] != X[2][t], Y[1][t] != Y[2][t]))
        s.add(Or(X[1][t] != X[3][t], Y[1][t] != Y[3][t]))
        s.add(Or(X[2][t] != X[3][t], Y[2][t] != Y[3][t]))

    safe = []
    for r in range (R):
        safe += safe_space (start_x[r], start_y[r], local_range, map) # need to change safe space
    safe += covered

    # local bound constraints
    for r in range (R):
        for t in range (T):
            s.add (Or ([And (X[r][t] == x, Y[r][t] == y) for (x, y) in safe]))

    # cover as many visible space as possible
    count = 0
    for (x,y) in visible:
        s.add (Implies (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)]), Re[count] == old_w * count))
        s.add (Implies (Not (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)])), Re[count] == 2 * num_visible))
        count += 1

    # if lost then the robot should gravitate towards the visible grids
    for r in range (R):
        for t in range (T):
            for (x,y) in safe:
                cost = 0
                for i in range (num_visible):
                    cost += (abs (visible[i][0] - x) + abs (visible[i][1] - y))
                cost = visible_w * int (cost / num_visible)
                s.add (Implies (And (X[r][t] == x, Y[r][t] == y), S[r][t] == cost))

    for r in range (R):
        for t in range (T):
            s.add (Implies (Or ([And (X[r][t] == x, Y[r][t] == y) for (x,y) in covered]), NC[r][t] == not_covered_w))
            s.add (Implies (Not (Or ([And (X[r][t] == x, Y[r][t] == y) for (x,y) in covered])), NC[r][t] == 1000))

    h = s.minimize (total_c)

    if str (s.check ()) == 'unsat':
        print ("UNSAT!")
        break

    model = s.model ()

    for r in range (R):
        for t in range (T):
            print (model[X[r][t]], model[Y[r][t]])
        print ('-------------------')

    for r in range (R):
        start_x[r] = int (str (model[X[r][T-1]]))
        start_y[r] = int (str (model[Y[r][T-1]]))

    # remove the grids already covered
    for r in range (R):
        for t in range (T):
            i = int (str (model[Y[r][t]]))
            j = int (str (model[X[r][t]]))
            local_map[i][j] = 1.0
            map[i][j] = 1.0
            covered.append ((j,i))
            if ((j,i) in visible):
                visible.remove ((j,i))

    # fill the visible array from the trajectory that robot followed
    for r in range (R):
        for t in range (T):
            i = int (str (model[Y[r][t]]))
            j = int (str (model[X[r][t]]))
            fill_obstacle_tocover (j, i, obstacles, visible, local_range, map)
            s = str (j) + " " + str (i) + "\n"
            files[r].write (s)
            files[r].flush ()

    print (local_map)

    num_covered = np.count_nonzero (map == 0.0) + np.count_nonzero (map == 1.0)

    print ("no. of cells covered:", num_covered)
    print ("horizon:", k)

    k += 1

for f in files:
    f.close ()
