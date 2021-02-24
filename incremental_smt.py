#!/usr/bin/env python3

from z3 import *
import numpy as np

def fill_obstacle_tocover (x_, y_, obstacles, visible, l, map):
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    x_l = max (0, x_-l)
    x_h = min (dimension_x, x_+l+1)
    y_l = max (0, y_-l)
    y_h = min (dimension_y, y_+l+1)
    for x in range (x_l, x_h):
        for y in range (y_l, y_h):
            if (map[y][x] == 0.0) and (x,y) not in obstacles:
                obstacles.append ((x,y))
            elif (map[y][x] == 0.5) and (x,y) not in visible:
                visible.append ((x,y))

def safe_space (x_, y_, l, map):
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    x_l = max (0, x_-l)
    x_h = min (dimension_x, x_+l+1)
    y_l = max (0, y_-l)
    y_h = min (dimension_y, y_+l+1)
    arr = []
    for x in range (x_l, x_h):
        for y in range (y_l, y_h):
            if map[y][x] == 0.5 or map[y][x] == 1.0:
                arr.append ((x,y))
    return arr

map = np.array ([[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.0,0.0,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.0,0.0,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5],
                 [0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5]])

local_map = np.zeros_like (map)

print (map.shape)

num_covered = 0
total = map.shape[0] * map.shape[1]
R = 2
local_range = 1

start_x_0 = 0 # 0
start_y_0 = 0 # 0

start_x_1 = 0 # 0
start_y_1 = 2 # 0

dimension_x = map.shape[1]
dimension_y = map.shape[0]

obstacles = []
visible = []
covered = []

fill_obstacle_tocover (start_x_0, start_y_0, obstacles, visible, local_range, map)
fill_obstacle_tocover (start_x_1, start_y_1, obstacles, visible, local_range, map)

k = 0

while num_covered < total:
    print ("robot0:", start_x_0, start_y_0)
    print ("robot1:", start_x_1, start_y_1)

    num_visible = len (visible)

    T = 4
    s = Optimize ()
    total_c = Int ('total_c')
    total_re = Int ('total_re')

    X = [[Int("x_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    Y = [[Int("y_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    P = [[Int("p_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
    S = [[Int("s_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

    Re = [Int("re_%s" % (j)) for j in range(num_visible)]

    C = [[Int("c_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

    s.add (total_c == Sum (Re + S[0] + S[1] + C[0] + C[1]))

    s.add (And (X[0][0] == start_x_0, Y[0][0] == start_y_0))
    s.add (Or (X[0][T-1] != start_x_0, Y[0][T-1] != start_y_0))

    s.add (And (X[1][0] == start_x_1, Y[1][0] == start_y_1))
    s.add (Or (X[1][T-1] != start_x_1, Y[1][T-1] != start_y_1))

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
            s.add (Or (C[r][t] == 2, C[r][t] == 5))
            s.add (And (S[r][t] >= 0, S[r][t] <= 32))

    for r in range (R):
        for t in range (T-1):
            s.add(Implies(P[r][t] == 0, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t], C[r][t] == 2))) # same
            s.add(Implies(P[r][t] == 1, And(X[r][t+1] == X[r][t]+1, Y[r][t+1] == Y[r][t], C[r][t] == 5))) # right
            s.add(Implies(P[r][t] == 2, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]+1, C[r][t] == 5))) # up
            s.add(Implies(P[r][t] == 3, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]-1, C[r][t] == 5))) # down
            s.add(Implies(P[r][t] == 4, And(X[r][t+1] == X[r][t]-1, Y[r][t+1] == Y[r][t], C[r][t] == 5))) # left

    safe = safe_space (start_x_0, start_y_0, local_range, map) # need to change safe space
    safe += safe_space (start_x_1, start_y_1, local_range, map)
    safe += visible

    # local bound constraints
    for r in range (R):
        for t in range (T):
            s.add (Or ([And (X[r][t] == x, Y[r][t] == y) for (x, y) in safe]))

    # cover as many visible space as possible
    count = 0
    for (x,y) in visible:
        s.add (Implies (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)]), Re[count] == count))
        s.add (Implies (Not (Or ([And (X[r][t] == x, Y[r][t] == y) for r in range (R) for t in range (T)])), Re[count] == 2 * num_visible))
        count += 1

    # if lost then the robot should gravitate towards the visible grids
    for r in range (R):
        for t in range (T):
            for (x,y) in safe:
                s.add (Implies (And (X[r][t] == x, Y[r][t] == y), S[r][t] == (abs (visible[0][0] - x) + abs (visible[0][1] - y))))

    h = s.minimize (total_c)

    if str (s.check ()) == 'unsat':
        print ("UNSAT!")
        break

    model = s.model ()

    for r in range (R):
        for t in range (T):
            print (model[X[r][t]], model[Y[r][t]])
        print ('-------------------')

    start_x_0 = int (str (model[X[0][T-1]]))
    start_y_0 = int (str (model[Y[0][T-1]]))

    start_x_1 = int (str (model[X[1][T-1]]))
    start_y_1 = int (str (model[Y[1][T-1]]))

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

    print (local_map)

    num_covered = np.count_nonzero (map == 0.0) + np.count_nonzero (map == 1.0)

    print ("no. of cells covered:", num_covered)
    print ("horizon:", k)

    k += 1
