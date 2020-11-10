#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import cv2
import datetime
import numpy as np
import math
import tf
import os
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Bool

def callback(robot_pose):
    h = Header()
    h.stamp = robot_pose.header.stamp
    h.frame_id = 'confusion'
    output = PoseStamped()
    output.header = h
    output.pose.position.x = robot_pose.pose.position.x + 0
    output.pose.position.y = robot_pose.pose.position.y + 0
    output.pose.position.z = robot_pose.pose.position.z + 0

    output.pose.orientation.x = robot_pose.pose.orientation.x + 0
    output.pose.orientation.y = robot_pose.pose.orientation.y + 0
    output.pose.orientation.z = robot_pose.pose.orientation.z + 0
    output.pose.orientation.w = robot_pose.pose.orientation.w + 0

    pub.publish(output)

def bool_cb(flg):
    pass

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber(("/mavros/local_position/pose", PoseStamped, callback, queue_size=10)
        #rospy.Subscriber("/AR/confution_pose/control", Bool, bool_cb, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'pose_confusion'
        pub = rospy.Publisher("/AR/confution_pose", PoseStamped, queue_size=10)
        error = Pose()
        main()
    except KeyboardInterrupt:
        pass