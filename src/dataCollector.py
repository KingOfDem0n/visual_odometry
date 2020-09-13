#!/usr/bin/env python

import numpy as np
import pandas as pd
import datetime
import math
import os

import rospy
from visual_odometry.srv import getRealSenseOdom
from visual_odometry.srv import V_Odometry
from visual_odometry.srv import turtlebotOdom
from std_msgs.msg import Empty

from tf.transformations import euler_from_quaternion

class dataCollector(object):
    def __init__(self):
        self.header = pd.MultiIndex.from_product([["T265", "Turtlebot", "Estimate"],
                                                  ["Time (sec)", "X (m)", "Y (m)", "Z (m)", "Row (rad)", "Pitch (rad)", "Yaw (rad)", "Yaw (deg)"]])
        self.T265Odom = np.zeros((1, 8))
        self.turtleOdom = np.zeros((1, 8))
        self.estOdom = np.zeros((1, 8))
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

    def unpackOdometryMsg(self, odom, sensor):
        if odom is None:
            return np.array([np.nan]*7)
        offsets = {"C": 0.087,
                   "T": 0.14}

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z

        q0 = odom.pose.pose.orientation.x
        q1 = odom.pose.pose.orientation.y
        q2 = odom.pose.pose.orientation.z
        q3 = odom.pose.pose.orientation.w
        rpy = euler_from_quaternion([q0, q1, q2, q3])

        time_lapse = (odom.header.stamp - self.startTime).to_sec()

        if sensor == "C":
            x = x - offsets["C"] + offsets["C"]*math.cos(rpy[2])
            y = y + offsets["C"]*math.sin(rpy[2])
        elif sensor == "T":
            x = x + offsets["T"] - offsets["T"]*math.cos(rpy[2])
            y = y - offsets["T"]*math.sin(rpy[2])

        deg_yaw = rpy[2]*180/math.pi

        if deg_yaw < 0:
            deg_yaw += 360

        return np.array([time_lapse, x, y, z, rpy[0], rpy[1], rpy[2], deg_yaw])

    def collect(self):
        _T265Odom = self._getT265Pose()
        _turtleOdom = self._getTurtlebotPose()
        _estOdom = self._getEstimatePose()

        upacked = self.unpackOdometryMsg(_T265Odom, sensor="T")
        self.T265Odom = np.vstack((self.T265Odom, upacked))

        upacked = self.unpackOdometryMsg(_turtleOdom, sensor="t")
        self.turtleOdom = np.vstack((self.turtleOdom, upacked))

        upacked = self.unpackOdometryMsg(_estOdom, sensor="C")
        self.estOdom = np.vstack((self.estOdom, upacked))

    def save(self, file):
        T265_df = pd.DataFrame(self.T265Odom)
        turtle_df = pd.DataFrame(self.turtleOdom)
        est_df = pd.DataFrame(self.estOdom)

        df = (T265_df.join(turtle_df, lsuffix='_T265', rsuffix='_turtlebot')).join(est_df, rsuffix='_estimate')

        df.to_excel(file, header=self.header, index=False)

def Stop(msg):
    global flag

    flag = False

if __name__ == "__main__":
    rospy.init_node('data_Collection')
    rospy.Subscriber("/done", Empty, Stop)

    flag = True
    dataset = dataCollector()
    rate = rospy.Rate(10)  # 10 Hz

    while flag and not rospy.is_shutdown():
        dataset.collect()
        rate.sleep()

    date = datetime.datetime.now()
    dataset.save(os.path.join("/home/pete/catkin_ws/src/visual_odometry/dataset", "plus-{}-{}-{}-{}-{}.xlsx".format(date.year, date.month, date.day, date.hour, date.minute)))
