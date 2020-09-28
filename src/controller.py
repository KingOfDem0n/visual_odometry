#!/usr/bin/env python

from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def stop(pub):
    cmd = Twist()
    pub.publish(cmd)

def forward(x, pub, end_stop=True):
    cmd = Twist()
    cmd.linear.x = 0.2

    start_time = rospy.Time.now()
    # time = start_time
    duration = rospy.Duration.from_sec((x/cmd.linear.x)*1.35) #1.35
    pub.publish(cmd)
    rospy.sleep(duration)
    # while (time - start_time).to_sec() < duration:
    #     pub.publish(cmd)
    #     time = rospy.Time.now()
    #     rate.sleep()

    if end_stop:
        stop(pub)

def rotate(deg, pub, end_stop=True):
    cmd = Twist()
    if deg > 0:
        cmd.angular.z = 0.5
    else:
        cmd.angular.z = -0.5

    theta = abs(deg * math.pi / 180.0)
    start_time = rospy.Time.now()
    # time = start_time
    duration = rospy.Duration.from_sec((theta / 0.5)*1.9) #1.9

    pub.publish(cmd)
    rospy.sleep(duration)

    # while (time - start_time).to_sec() < duration:
    #     pub.publish(cmd)
    #     time = rospy.Time.now()
    #     rate.sleep()

    if end_stop:
        stop(pub)

def curve(pub, end_stop=True):
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = 0.4

    duration = rospy.Duration.from_sec(((math.pi/2.0)/cmd.angular.z)*1.4)
    pub.publish(cmd)
    rospy.sleep(duration)

    if end_stop:
        stop(pub)

def roundBox(pub):
    forward(0.5, pub)
    rospy.sleep(1)
    curve(pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    curve(pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    curve(pub)
    rospy.sleep(1)

    forward(0.5, pub)
    rospy.sleep(1)
    curve(pub)
    rospy.sleep(1)

def circle(pub):
    cmd = Twist()
    cmd.linear.x = 0.2
    cmd.angular.z = 0.4

    duration = rospy.Duration.from_sec(((math.pi/2.0)/cmd.angular.z)*5)
    pub.publish(cmd)
    rospy.sleep(duration)

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
    rotate(90, pub)
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
    rotate(90, pub)
    rospy.sleep(1)
    forward(0.5, pub)
    rospy.sleep(1)

    rotate(90, pub)
    rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node('VO_controller')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    done = rospy.Publisher("/done", Empty, queue_size=1)

    rate = rospy.Rate(100) # 100 Hz

    rospy.sleep(5.)
    # forward(0.5, pub)
    # rotate(-90, pub)
    # curve(pub)
    # boxMovement(pub)
    # plusMovement(pub)
    # roundBox(pub)
    circle(pub)

    done.publish(Empty())
