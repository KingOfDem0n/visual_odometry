#!/usr/bin/env python

from __future__ import print_function

from visual_odometry.srv import turtlebotOdom, turtlebotOdomResponse
import rospy
from nav_msgs.msg import Odometry

def callback_server(req):
    response = turtlebotOdomResponse()
    response.turtlebotOdom = odom

    return response


def callback_odom(msg):
    global odom

    odom = msg


if __name__ == "__main__":
    odom = Odometry()

    rospy.init_node('turtlebotOdom_server')
    rospy.Service('turtlebotOdom', turtlebotOdom, callback_server)
    rospy.Subscriber("/odom", Odometry, callback_odom)

    rospy.spin()
