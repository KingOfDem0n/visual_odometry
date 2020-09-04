#!/usr/bin/env python

from __future__ import print_function

import rospy
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2 as cv
from visual_odometry.srv import getImage

def getImages():
    # rospy.loginfo("Waiting for imageGrabber server...")
    rospy.wait_for_service('imageGrabber')
    try:
        server = rospy.ServiceProxy('imageGrabber', getImage)
        return server()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    data = getImages()
    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(data.rgb, desired_encoding='bgr8')
    _point3D = np.array(list(pc2.read_points(data.point))).reshape((data.point.height, data.point.width, 3))
    point3D = np.array(list(pc2.read_points(data.point, uvs=[(300, 350)])))
    print(point3D)
    print(_point3D[350, 300])
