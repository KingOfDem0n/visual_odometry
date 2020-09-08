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
from visual_odometry.srv import getRealSenseOdom

def getTruePose():
    rospy.wait_for_service('getRealSenseOdom')
    try:
        server = rospy.ServiceProxy('getRealSenseOdom', getRealSenseOdom)
        return server()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def quat2euler(orientation):
    q0 = orientation.w
    q1 = orientation.x
    q2 = orientation.y
    q3 = orientation.z

    row = math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
    pitch = math.asin(2*(q0*q1 - q3*q1))
    yaw = math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))

    return [row, pitch, yaw]

def stop(pub):
    cmd = Twist()
    pub.publish(cmd)

def forward(x, stereo, pub):
    cmd = Twist()
    cmd.linear.x = 0.2

    start_R = cur_R_matrix.copy()
    start_t = start_R.T.dot(np.array([[current_Pose.linear.x],
                                      [current_Pose.linear.y],
                                      [current_Pose.linear.z]]))


    cur_t = start_t

    # print(np.abs(start_t[0] - cur_t[0]) < x)
    while np.abs(start_t[0] - cur_t[0]) < x:
        # print(np.abs(start_t - cur_t))

        pub.publish(cmd)
        getPose(stereo)
        cur_t = start_R.T.dot(np.array([[current_Pose.linear.x],
                                        [current_Pose.linear.y],
                                        [current_Pose.linear.z]]))

        assert np.any(np.isnan(cur_t)), "THere is a nan value in forward function!"

    stop(pub)

def rotate(deg, stereo, pub):
    cmd = Twist()
    if deg > 0:
        cmd.angular.z = 0.5
    else:
        cmd.angular.z = -0.5
    theta = abs(deg*math.pi/180.0)
    start = abs(current_Pose.angular.z)
    current = start
    prev = start
    total = 0
    # print("theta: {}, start: {}".format(theta, start))
    # print("Current pose: {}".format(current_Pose.angular.z))

    while total < theta:
        getPose(stereo)
        pub.publish(cmd)
        diff = abs(abs(current) - abs(prev))
        total += diff
        prev = current
        current = abs(current_Pose.angular.z)
        # print(total*180.0/math.pi)

    stop(pub)

def updatePose(R_matrix, R_rpy, t):
    global current_Pose, true_Pose, cur_R_matrix, stats

    current_Pose.linear.x = t[0].squeeze() - offsets["C"] + offsets["C"]*math.cos(-R_rpy[1].squeeze())
    current_Pose.linear.y = t[1].squeeze() + offsets["C"]*math.sin(-R_rpy[1].squeeze())
    current_Pose.linear.z = t[2].squeeze()
    current_Pose.angular.x = 0.0
    current_Pose.angular.y = 0.0
    current_Pose.angular.z = R_rpy[1].squeeze()

    data = getTruePose().groundTruth

    true_Pose.linear.x = data.pose.pose.position.x + offsets["T"] - offsets["T"]*math.cos(-R_rpy[1].squeeze())
    true_Pose.linear.y = data.pose.pose.position.y - offsets["T"]*math.sin(-R_rpy[1].squeeze())
    true_Pose.linear.z = data.pose.pose.position.z

    cur_R_matrix = R_matrix.copy()

    stats["x_est"].append(current_Pose.linear.x)
    stats["y_est"].append(current_Pose.linear.y)
    stats["theta_est"].append(current_Pose.angular.z)
    stats["x_true"].append(true_Pose.linear.x)
    stats["y_true"].append(true_Pose.linear.y)

    true_euler = quat2euler(data.pose.pose.orientation)
    stats["theta_true"].append(true_euler[2])

    diff = np.array([stats["x_est"][-1] - stats["x_true"][-1], stats["y_est"][-1] - stats["y_true"][-1]])
    stats["error"].append(np.sqrt(np.sum(diff**2)))

def convert(R, t):
    translation = np.array([[0.0, 0.0, 1.0],
                            [1.0, 0.0, 0.0],
                            [0.0,-1.0, 0.0]])

    t = translation.dot(t)
    _R, _ = cv.Rodrigues(R)

    # test = cur_R_matrix.dot(R)

    R = np.array([[math.cos(_R[1]), -math.sin(_R[1]), 0],
                  [math.sin(_R[1]), math.cos(_R[1]), 0],
                  [0, 0, 1]])

    # print(test)
    # print(R)

    return R, _R, t

def getPose(stereo):
    R_matrix, t, gray = stereo.nextFrame()
    updatePose(*convert(R_matrix, t))

def boxMovement(stereo, pub):
    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(90, stereo, pub)
    time.sleep(1)

    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(90, stereo, pub)
    time.sleep(1)

    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(90, stereo, pub)
    time.sleep(1)

    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(90, stereo, pub)
    time.sleep(1)

def plusMovement(stereo, pub):
    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(180, stereo, pub)
    time.sleep(1)
    forward(0.5, stereo, pub)
    time.sleep(1)

    rotate(90, stereo, pub)
    time.sleep(1)

    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(180, stereo, pub)
    time.sleep(1)
    forward(0.5, stereo, pub)
    time.sleep(1)

    rotate(90, stereo, pub)
    time.sleep(1)

    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(180, stereo, pub)
    time.sleep(1)
    forward(0.5, stereo, pub)
    time.sleep(1)

    rotate(90, stereo, pub)
    time.sleep(1)

    forward(0.5, stereo, pub)
    time.sleep(1)
    rotate(180, stereo, pub)
    time.sleep(1)
    forward(0.5, stereo, pub)
    time.sleep(1)

    rotate(90, stereo, pub)
    time.sleep(1)

def circleMovement(stereo, pub):
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = 0.5
    count = 0

    while True:
        pub.publish(cmd)
        getPose(stereo)

        time.sleep(0.001)
        count += 1
        if count > 500:
            break

    stop(pub)

def displayStat(stats):
    position_figure = plt.figure(1)
    plt.title("Map")
    position_axes = position_figure.add_subplot(1, 1, 1)
    position_axes.set_aspect('equal', adjustable='box')

    position_axes.scatter(stats["x_est"], stats["y_est"], c="blue", s=10)
    position_axes.scatter(stats["x_true"], stats["y_true"], c="red", s=10)
    
    plt.figure(2)
    plt.title("X-axis")
    plt.xlabel("Frames")
    plt.ylabel("Meters")
    plt.plot(stats["x_est"], c="blue")
    plt.plot(stats["x_true"], c="red")

    plt.figure(3)
    plt.title("Y-axis")
    plt.xlabel("Frames")
    plt.ylabel("Meters")
    plt.plot(stats["y_est"], c="blue")
    plt.plot(stats["y_true"], c="red")

    plt.figure(4)
    plt.title("Trajectory Error")
    plt.xlabel("Frames")
    plt.ylabel("Meters")
    plt.plot(stats["error"], c="blue")

    plt.show()

if __name__ == "__main__":
    rospy.init_node('VO_driver')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    true_Pose = Twist()
    current_Pose = Twist()
    cur_R_matrix = np.eye(3)

    stereo = Stereo()
    stereo.initialize()

    stats = {"x_est": [],
             "y_est": [],
             "theta_est": [],
             "x_true": [],
             "y_true": [],
             "theta_true": [],
             "error": []}
    # TODO: Measure offset distance for both cameras
    offsets = {"C": 0,
               "T": 0}

    try:
        forward(5, stereo, pub)
        displayStat(stats)

    except KeyboardInterrupt:
        print('interrupted!')

    finally:
        try:
            plt.close('all')
        except:
            pass
