#!/usr/bin/env python3

import numpy as np
import cv2

from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

def plot (image, origin):

    height = image.shape[0]
    width = image.shape[1]

    covered = []
    uncovered = []

    origin = (int (origin[1]/10), int ((height-origin[0])/10))

    for i_ in range (int (height/10)):
        for j_ in range (int (width/10)):
            i = min (i_ * 10 + 5, height)
            j = min (j_ * 10 + 5, width)
            if (np.all (image[i,j] == np.array([0,0,0]))):
                x = j_
                y = (int (height/10) - i_ - 1)
                uncovered.append ((x, y))
            else:
                x = j_
                y = (int (height/10) - i_ - 1)
                covered.append ((x, y))

    covered = np.array (covered)
    uncovered = np.array (uncovered)

    # covered = covered - np.array (origin)
    # uncovered = uncovered - np.array (origin)

    fig, ax = plt.subplots (figsize=(int (width/10), int (height/10)))
    ax.set (xlim=(0,int (width/10)), ylim=(0, int (height/10)))
    # ax.set (xlim=(0 - origin[0],int (width/10) - origin[0]), ylim=(0 - origin[1], int (height/10) - origin[1]))
    # ax.axis ('off')
    alpha = 1

    for c in covered:
        color = 'red'
        r = Rectangle ((c[0],c[1]), 1, 1, linewidth=1, edgecolor='black', facecolor=color, alpha=alpha)
        ax.add_patch (r)

    for u in uncovered:
        color = 'gray'
        r = Rectangle ((u[0],u[1]), 1, 1, linewidth=1, edgecolor='black', facecolor=color, alpha=alpha)
        ax.add_patch (r)

    plt.show ()

image = cv2.imread ('results/finalResult.jpg')
origin = (1113, 867)
plot (image, origin)
