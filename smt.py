from z3 import *

s = Optimize()
T = 13
R = 4
total_c = Int ('total_c')

X = [[Int("x_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
Y = [[Int("y_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
P = [[Int("p_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

C = [[Int("c_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
s.add(total_c == Sum(C[0] + C[1] + C[2] + C[3]))

dimension_x = 7
dimension_y = 7

# Start Positions
s.add(X[0][0] == 0)
s.add(Y[0][0] == 0)

s.add(X[1][0] == 0)
s.add(Y[1][0] == 1)

s.add(X[2][0] == 1)
s.add(Y[2][0] == 0)

s.add(X[3][0] == 1)
s.add(Y[3][0] == 1)

obst = [(2,0), (3,0), (1,2), (3,2), (1,4), (2,4)]
for r in range(R):
    for t in range(0,T):
        for ob in obst:
            s.add(Or(X[r][t] != ob[0], Y[r][t] != ob[1]))

# Obstacle avoidance
for r in range(R):
    for t in range(0,T):

        # stay within bounds
        s.add(And (X[r][t] < dimension_x, X[r][t] >= 0))
        s.add(And (Y[r][t] < dimension_y, Y[r][t] >= 0))
        s.add(And (P[r][t] <= 4, P[r][t] >= 0))

        s.add (Or (C[r][t] == 1, C[r][t] == 2))

# collision AVOIDANCE
for t in range(0, T):
    s.add(Or(X[0][t] != X[1][t], Y[0][t] != Y[1][t]))
    s.add(Or(X[0][t] != X[2][t], Y[0][t] != Y[2][t]))
    s.add(Or(X[0][t] != X[3][t], Y[0][t] != Y[3][t]))
    s.add(Or(X[1][t] != X[2][t], Y[1][t] != Y[2][t]))
    s.add(Or(X[1][t] != X[3][t], Y[1][t] != Y[3][t]))
    s.add(Or(X[2][t] != X[3][t], Y[2][t] != Y[3][t]))

# full coverage condition
for x in range(0, dimension_x):
    for y in range(0, dimension_y):
        if ((x,y) not in obst):
            s.add(Or([And(X[r][t] == x, Y[r][t] == y) for r in range(0, R) for t in range (0, T)]))

# Motion primitives
for r in range(0, R):
    for t in range(T-1):
        s.add(Implies(P[r][t] == 0, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t], C[r][t] == 1))) # same
        s.add(Implies(P[r][t] == 1, And(X[r][t+1] == X[r][t]+1, Y[r][t+1] == Y[r][t], C[r][t] == 2))) # right
        s.add(Implies(P[r][t] == 2, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]+1, C[r][t] == 2))) # up
        s.add(Implies(P[r][t] == 3, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]-1, C[r][t] == 2))) # down
        s.add(Implies(P[r][t] == 4, And(X[r][t+1] == X[r][t]-1, Y[r][t+1] == Y[r][t], C[r][t] == 2))) # left

h = s.minimize(total_c)

print ("Whether the model is satisfiable?: ", s.check())
print ("============ Solution ================")
model = s.model()
print (model)
print (model[total_c])

def generate_plan ():
    for r in range (R):
        filename = 'robot' + str (r) + '.plan'
        f = open (filename, 'w+')
        for t in range (T):
            coord = str (str (model[X[r][t]]) + " " + str (model[Y[r][t]]) + " 2\n")
            f.write (coord)
            f.flush ()
        f.close()

generate_plan ()
