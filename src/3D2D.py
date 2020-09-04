#!/usr/bin/env python

from __future__ import print_function

# Python imports
import cv2 as cv
import numpy as np
import math
import os
import matplotlib.pyplot as plt
from Stereo import Stereo
import time

import rospy
from geometry_msgs.msg import Twist

# TODO: Create a way to measure error in trajectory estimations. Try getting information from IMU or use a real physical object such as a ruler
#       to gauge the estimation.

# NOTE: Run time might still be an issue. Will need to test it.
#           Perhaps try to only get new keypoints when only certain number of keypoints remains.
#           It could be slow due to the need to display plot as well. Keep that in mind.

def forward(x, stereo, pub):
    cmd = Twist()
    cmd.linear.x = 0.2

    start_R = cur_R_matrix.copy()
    start_t = start_R.dot(np.array([[current_Pose.linear.x],
                                    [current_Pose.linear.y],
                                    [current_Pose.linear.z]]))

    cur_t = start_R.dot(np.array([[current_Pose.linear.x],
                                  [current_Pose.linear.y],
                                  [current_Pose.linear.z]]))

    while np.abs(start_t[0] - cur_t[0]) < x:
        print(np.abs(start_t[0] - cur_t[0]))
        pub.publish(cmd)
        getPose(stereo)
        cur_t = start_R.dot(np.array([[current_Pose.linear.x],
                                      [current_Pose.linear.y],
                                      [current_Pose.linear.z]]))

    cmd.linear.x = 0.0
    pub.publish(cmd)

def rotate(deg, stereo, pub):
    cmd = Twist()
    if deg > 0:
        cmd.angular.z = 0.5
    else:
        cmd.angular.z = -0.5
    theta = abs(deg*math.pi/180.0)
    start = abs(current_Pose.angular.z)
    goal = (start + theta) - 2*math.pi*((start + theta)//(2*math.pi))
    print("theta: {}, start: {}".format(theta, start))
    print("Current pose: {}".format(current_Pose.angular.z))

    while np.abs(start - abs(current_Pose.angular.z)) < theta:
        print(np.abs(start - abs(current_Pose.angular.z))*180.0/math.pi)
        # print(current_Pose.angular.z)
        # print(cur_R_matrix)
        getPose(stereo)
        pub.publish(cmd)

    # print("Final angle {}".format(current_Pose.angular.z))
    # print(cur_R_matrix)
    cmd.angular.z = 0.0
    pub.publish(cmd)

def updatePose(R_matrix, R_rpy, t):
    global current_Pose, cur_R_matrix

    current_Pose.linear.x = t[0].squeeze()
    current_Pose.linear.y = t[1].squeeze()
    current_Pose.linear.z = t[2].squeeze()
    current_Pose.angular.x = R_rpy[1].squeeze()
    current_Pose.angular.y = -R_rpy[2].squeeze()
    current_Pose.angular.z = R_rpy[1].squeeze()

    cur_R_matrix = R_matrix.copy()

def convert(R, t):
    translation = np.array([[0.0, 0.0, 1.0],
                            [1.0, 0.0, 0.0],
                            [0.0,-1.0, 0.0]])

    t = translation.dot(t)
    # R = translation.dot(R)
    _R, _ = cv.Rodrigues(R)
    # print(R)
    # print(_R[1])
    # if _R[1] < 0:
    #     _R[1] += math.pi * 2
    # print(_R[1])
    return R, _R, t

def getPose(stereo):
    R_matrix, t, gray = stereo.nextFrame()
    # R_rpy, R_matrix, t =convert(R_matrix, t)
    updatePose(*convert(R_matrix, t))


# TODO: Fix rotation method.

if __name__ == "__main__":
    rospy.init_node('VO_driver')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    current_Pose = Twist()
    cur_R_matrix = np.eye(3)

    position_figure = plt.figure()
    position_axes = position_figure.add_subplot(1, 1, 1)
    position_axes.set_aspect('equal', adjustable='box')

    stereo = Stereo()
    stereo.initialize()

    try:
        rotate(-90, stereo, pub)
        time.sleep(2)
        rotate(-90, stereo, pub)
        # forward(1.0, stereo, pub)
        # time.sleep(2)
        # forward(1.0, stereo, pub)
        # position_axes.scatter(x_est, y_est, c="blue")
        # plt.figure(2)
        # plt.plot(time_track)
        # plt.plot(time_avg)
        # plt.pause(.01)

        # cv.imshow('Left Frame', gray)
        # cv.waitKey(1)

    except KeyboardInterrupt:
        print('interrupted!')

    finally:
        try:
            cv.destroyWindow("Left Frame")
            plt.close('all')
        except:
            pass
