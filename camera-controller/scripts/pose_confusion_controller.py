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
from geometry_msgs.msg import PoseStamped

def callback(robot_pose):
    h = Header()
    h.stamp = robot_pose.header
    h.frame_id = 'confusion'
    output = PoseStamped()
    output.header = h
    output.pose = robot_pose.pose.pose
    pub.publish(output)

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'pose_confusion_controller'
        pub = rospy.Publisher("/AR/confution_pose/control", PoseStamped, queue_size=10)
        main()
    except KeyboardInterrupt:
        pass