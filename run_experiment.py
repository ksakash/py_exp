#!/usr/bin/env python3

import os

robots = [2]#, 4, 8, 16]
dimensions = [4]#, 32, 64, 100]
count = 1

if not os.path.isdir ('experiments'):
    os.mkdir ('experiments')

for d in dimensions:
    for r in robots:
        filename = 'experiments/r_' + str (r) + '_d_' + str (d)
        if not os.path.isfile (filename):
            os.system ('touch {}'.format (filename))
        os.system ('cat /dev/null > {}'.format (filename))
        for c in range (count):
            cmd = './incremental_smt.py -r {} -d {} -f {} -c {} -m {}'.format (r, d, filename, c, 3)
            os.system (cmd)