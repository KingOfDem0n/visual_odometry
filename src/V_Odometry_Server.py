#!/usr/bin/env python

from __future__ import print_function

# Python imports
import cv2 as cv
import numpy as np
from Stereo import Stereo

import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Odometry
from visual_odometry.srv import V_Odometry, V_OdometryResponse

def updatePose(stereo):
    global odom

    R_matrix, t, _ = stereo.nextFrame()

    R_rpy, t = convert(R_matrix, t)

    q = quaternion_from_euler(R_rpy[0], R_rpy[1], R_rpy[2])

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    odom.pose.position.x = t[0].squeeze()
    odom.pose.position.y = t[1].squeeze()
    odom.pose.position.z = t[2].squeeze()
    odom.pose.orientation.x = q[0]
    odom.pose.orientation.y = q[1]
    odom.pose.orientation.z = q[2]
    odom.pose.orientation.w = q[3]

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

def callback_server(req):
    response = V_OdometryResponse()
    response.estimate = odom

    return response

if __name__ == "__main__":
    odom = Odometry()

    rospy.init_node('V_Odometry_Server')

    stereo = Stereo()
    stereo.initialize()

    while not rospy.is_shutdown():
        updatePose(stereo)
        rospy.Service('V_Odometry', V_Odometry, callback_server)


