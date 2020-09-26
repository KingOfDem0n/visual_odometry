#!/usr/bin/env python

from __future__ import print_function

import numpy as np

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def callback_odom(msg):
    twist_covariance = np.eye(6) * 10e300
    twist_covariance[0,0] = 0.1
    twist_covariance[1,1] = 0.0
    twist_covariance[-1,-1] = 0.1

    pose_covariance = list(msg.pose.covariance)
    pose_covariance[28] = 0.05

    msg.pose.covariance = pose_covariance
    msg.twist.covariance = twist_covariance.reshape(36).tolist()

    odom_pub.publish(msg)

    rate.sleep()

def callback_imu(msg):
    covariance = np.eye(3) * 10e300
    covariance[1,1] = 0.1

    msg.linear_acceleration_covariance = covariance.reshape(9).tolist()

    imu_pub.publish(msg)

    rate.sleep()

if __name__ == "__main__":
    rospy.init_node('AddCovariance')
    imu_pub = rospy.Publisher('/mobile_base/sensors/imu_data_modified', Imu, queue_size=1)
    odom_pub = rospy.Publisher('/odom_modified', Odometry, queue_size=1)
    rate = rospy.Rate(25)

    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, callback_imu)

    rospy.spin()
