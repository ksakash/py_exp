#!/usr/bin/env python3

import pyclipper
import numpy as np

a = np.array ([(0,0), (10,0), (10,10), (0,10)])
subj = (a,)
c = np.array ([(5,5), (9,5), (9,9), (5,9)])
clip = (c)

pc = pyclipper.Pyclipper ()
pc.AddPath (clip, pyclipper.PT_CLIP, True)
pc.AddPaths (subj, pyclipper.PT_SUBJECT, True)

sol = pc.Execute (pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)

print (sol)
