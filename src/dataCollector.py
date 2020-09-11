#!/usr/bin/env python

import numpy as np
import pandas as pd
import os

import rospy
from visual_odometry.srv import getRealSenseOdom
from visual_odometry.srv import V_Odometry
from visual_odometry.srv import turtlebotOdom

from tf.transformations import euler_from_quaternion


class dataCollector(object):
    def __init__(self):
        self.header = pd.MultiIndex.from_product([["T265", "Turtlebot", "Estimate"],
                                                  ["Time (sec)", "X (m)", "Y (m)", "Z (m)", "Row (rad)", "Pitch (rad)", "Yaw (rad)"]])
        self.T265Odom = np.zeros((1, 7))
        self.turtleOdom = np.zeros((1, 7))
        self.estOdom = np.zeros((1, 7))
        self.startTime = rospy.Time.now()

    def _getTurtlebotPose(self):
        rospy.wait_for_service('turtlebotOdom')
        try:
            server = rospy.ServiceProxy('turtlebotOdom', turtlebotOdom)
            return server().turtlebotOdom
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _getT265Pose(self):
        rospy.wait_for_service('getRealSenseOdom')
        try:
            server = rospy.ServiceProxy('getRealSenseOdom', getRealSenseOdom)
            return server().T265Odom
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def _getEstimatePose(self):
        rospy.wait_for_service('V_Odometry')
        try:
            server = rospy.ServiceProxy('V_Odometry', V_Odometry)
            return server().estimate
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def unpackOdometryMsg(self, odom):
        x = odom.pose.position.x
        y = odom.pose.position.y
        z = odom.pose.position.z

        q0 = odom.pose.orientation.x
        q1 = odom.pose.orientation.y
        q2 = odom.pose.orientation.z
        q3 = odom.pose.orientation.w
        rpy = euler_from_quaternion([q0, q1, q2, q3])

        time_lapse = (odom.header.stamp - self.startTime).toSec()

        return np.array([time_lapse, x, y, z, rpy[0], rpy[1], rpy[2]])

    def collect(self):
        T265Odom = self._getT265Pose()
        turtleOdom = self._getTurtlebotPose()
        estOdom = self._getEstimatePose()

        upacked = self.unpackOdometryMsg(T265Odom)
        self.T265Odom = np.vstack((self.T265Odom, upacked))

        upacked = self.unpackOdometryMsg(turtleOdom)
        self.turtleOdom = np.vstack((self.turtleOdom, upacked))

        upacked = self.unpackOdometryMsg(estOdom)
        self.estOdom = np.vstack((self.estOdom, upacked))

    def save(self, file):
        T265_df = pd.DataFrame(self.T265Odom, index=False)
        turtle_df = pd.DataFrame(self.turtleOdom, index=False)
        est_df = pd.DataFrame(self.estOdom, index=False)

        df = (T265_df.join(turtle_df)).join(est_df)
        df.to_excel(file, header=self.header, index=False)
