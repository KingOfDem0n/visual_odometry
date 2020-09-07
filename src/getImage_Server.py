#!/usr/bin/env python

from __future__ import print_function

from visual_odometry.srv import getImage, getImageResponse
import rospy
from sensor_msgs.msg import Image, PointCloud2

def callback_server(req):
    response = getImageResponse()
    response.rgb = image
    response.point = point
    response.depth = depth
    return response

def callback_image(msg):
    global image

    image = msg

def callback_point(msg):
    global point

    point = msg

def callback_depth(msg):
    global depth

    depth = msg

if __name__ == "__main__":
    image = Image()
    depth = Image()
    point = PointCloud2()

    rospy.init_node('getImageAndPoints_server')
    rospy.Service('imageGrabber', getImage, callback_server)
    rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback_image)
    rospy.Subscriber("/camera/depth/image_rect", Image, callback_depth)
    rospy.Subscriber("/camera/depth/points", PointCloud2, callback_point)
    rospy.spin()
