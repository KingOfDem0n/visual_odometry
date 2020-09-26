#!/usr/bin/env python

from __future__ import print_function

# Python imports
import cv2 as cv
import numpy as np
import math
from Stereo import Stereo

import rospy
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

def updatePose(stereo):
    global odom, image

    offset = 0.087

    R_matrix, t, gray = stereo.nextFrame()
    image = bridge.cv2_to_imgmsg(gray, encoding="passthrough")

    R_rpy, t = convert(R_matrix, t)

    q = quaternion_from_euler(R_rpy[0], R_rpy[1], R_rpy[2])
    rpy = euler_from_quaternion([q[0], q[1], q[2], q[3]])

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    odom.pose.pose.position.x = t[0].squeeze() - offset + offset*math.cos(rpy[2])
    odom.pose.pose.position.y = t[1].squeeze() + offset*math.sin(rpy[2])
    odom.pose.pose.position.z = t[2].squeeze()
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]

    odom.pose.covariance = (np.eye(6) * 0.1).reshape(36).tolist()

    twist_covariance = np.eye(6) * 10e300
    twist_covariance[1,1] = 0.0

    odom.twist.covariance = twist_covariance.reshape(36).tolist()

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

if __name__ == "__main__":
    rospy.init_node('VO_Estimate')
    pub = rospy.Publisher('/VO_estimate', Odometry, queue_size=1)
    image_pub = rospy.Publisher('/whatIsee', Image, queue_size=1)
    rate = rospy.Rate(25)
    bridge = CvBridge()
    odom = Odometry()
    image = Image()

    stereo = Stereo()
    stereo.initialize()

    while not rospy.is_shutdown():
        updatePose(stereo)
        pub.publish(odom)
        image_pub.publish(image)
        rate.sleep()
