#!/usr/bin/env python

from __future__ import print_function

# Python imports
import cv2 as cv
import numpy as np
import math
import matplotlib.pyplot as plt
from Stereo import Stereo

import rospy
from nav_msgs.msg import Odometry

def convert(R, t):
    translation = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0]])

    t = translation.dot(t)
    _R, _ = cv.Rodrigues(R)

    row = _R[0]
    pitch = -_R[2]
    yaw = -_R[1]

    return (row, pitch, yaw), t

def callback(msg):
    global truth_x, truth_y, est_x, est_y, gray

    R_matrix, t, gray = stereo.nextFrame()
    R_rpy, t = convert(R_matrix, t)

    t[0] = t[0].squeeze() - offset + offset*math.cos(R_rpy[2])
    t[1] = t[1].squeeze() + offset*math.sin(R_rpy[2])

    est_x.append(t[0])
    est_y.append(t[1])
    truth_x.append(msg.pose.pose.position.x)
    truth_y.append(msg.pose.pose.position.y)

if __name__ == "__main__":
    gray = None
    stereo = Stereo()
    _,_, gray = stereo.initialize()

    offset = 0.087 # 0.087

    truth_x = []
    truth_y = []
    est_x = []
    est_y = []

    position_figure = plt.figure(1)
    plt.title("Map")
    position_axes = position_figure.add_subplot(1, 1, 1)
    position_axes.set_aspect('equal', adjustable='box')

    rospy.init_node("Live_2D_Map")
    rospy.Subscriber("/odom", Odometry, callback)

    while(True):
        if(len(est_x) == len(est_y) and len(truth_x) == len(truth_y)):
            position_axes.scatter(est_x, est_y, c="blue", s=10)
            position_axes.scatter(truth_x, truth_y, c="red", s=10)

        cv.imshow("Frame", gray)
        cv.waitKey(1)
        plt.pause(0.01)
