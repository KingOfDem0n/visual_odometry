#!/usr/bin/env python

from __future__ import print_function

# Python imports
from Stereo import Stereo

import rospy
from nav_msgs.msg import Odometry
from visual_odometry.srv import V_Odometry, V_OdometryResponse

def callback_odom(msg):
    global odom

    odom = msg

def callback_server(req):
    response = V_OdometryResponse()
    response.estimate = odom

    return response

if __name__ == "__main__":
    odom = Odometry()

    rospy.init_node('V_Odometry_Server')
    rospy.Subscriber("/VO_estimate", Odometry, callback_odom)
    rospy.Service('V_Odometry', V_Odometry, callback_server)

    rospy.spin()
