#!/usr/bin/env python

from __future__ import print_function

from visual_odometry.srv import combinedOdom, combinedOdomResponse
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback_server(req):
    response = combinedOdomResponse()
    response.combined_odom = odom

    return response


def callback_odom(msg):
    global odom

    odom = msg


if __name__ == "__main__":
    odom = PoseWithCovarianceStamped()

    rospy.init_node('combinedOdom_server')
    rospy.Service('combinedOdom', combinedOdom, callback_server)
    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback_odom)

    rospy.spin()
