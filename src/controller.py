#!/usr/bin/env python

from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def stop(pub):
    cmd = Twist()
    pub.publish(cmd)

def forward(x, pub):
    cmd = Twist()
    cmd.linear.x = 0.2

    time = rospy.Time.now()
    duration = x/0.2

    while (time - start_time).toSec() < duration:
        pub.publish(cmd)
        rate.sleep()

    stop(pub)

def rotate(deg, pub):
    cmd = Twist()
    if deg > 0:
        cmd.angular.z = 0.5
    else:
        cmd.angular.z = -0.5

    theta = abs(deg * math.pi / 180.0)
    time = rospy.Time.now()
    duration = theta / 0.5

    while (time - start_time).toSec() < duration:
        pub.publish(cmd)
        rate.sleep()

    stop(pub)

def boxMovement(pub):
    forward(0.5, pub)
    rospy.sleep(1)
    rotate(90, pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    rotate(90, pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    rotate(90, pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    rotate(90, pub)
    rospy.sleep(1)

def plusMovement(pub):
    forward(0.5, pub)
    rospy.sleep(1)
    rotate(180, pub)
    rospy.sleep(1)
    forward(0.5, pub)
    rospy.sleep(1)

    rotate(90, pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    rotate(180, pub)
    rospy.sleep(1)
    forward(0.5, pub)
    rospy.sleep(1)

    rotate(90, pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    rotate(180, pub)
    rospy.sleep(1)
    forward(0.5, pub)
    rospy.sleep(1)

    rotate(90, pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    rotate(180, pub)
    rospy.sleep(1)
    forward(0.5, pub)
    rospy.sleep(1)

    rotate(90, pub)
    rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node('VO_controller')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    done = rospy.Publisher("/done", Empty, queue_size=1)

    start_time = rospy.Time.now()
    rate = rospy.Rate(10) # 10 Hz

    boxMovement(pub)
    # plusMovement(pub)

    done.publish(Empty())


