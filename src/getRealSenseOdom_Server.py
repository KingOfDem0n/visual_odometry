#!/usr/bin/env python

from __future__ import print_function

from visual_odometry.srv import getRealSenseOdom, getRealSenseOdomResponse
import rospy
from nav_msgs.msg import Odometry

def callback_server(req):
    response = getRealSenseOdomResponse()
    response.T265Odom = odom

    return response

def callback_odom(msg):
    global odom

    odom = msg

if __name__ == "__main__":
    odom = Odometry()

    rospy.init_node('getRealSenseOdom_server')
    rospy.Service('getRealSenseOdom', getRealSenseOdom, callback_server)
    rospy.Subscriber("/camera/odom/sample", Odometry, callback_odom)

    rospy.spin()
