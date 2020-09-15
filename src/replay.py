#!/usr/bin/env python

from __future__ import print_function

import cv2 as cv
import numpy as np
import os
import matplotlib.pyplot as plt
from Stereo import Stereo

path = os.path.join(os.getcwd(), "..", "dataset", "image_sequences", "box")
img = cv.imread(os.path.join(path, '{:06d}.png'.format(0)))
count = 1

stereo = Stereo()
stereo.initialize(img)

x = []
y = []

position_figure = plt.figure(1)
plt.title("Map")
position_axes = position_figure.add_subplot(1, 1, 1)
position_axes.set_aspect('equal', adjustable='box')

while True:
    img = cv.imread(os.path.join(path, '{:06d}.png'.format(count)))
    R, t, _ = stereo.nextFrame(img)
    x.append(t[0])
    y.append(t[1])
    position_axes.scatter(x, y, c="blue", s=10)
    cv.imshow("Image", img)
    count += 1
    if cv.waitKey(1) & 0xFF == ord('q'):
        cv.destroyAllWindows()
        break
