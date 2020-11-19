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
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Point, Quaternion

def mini2_cb(robot_pose):
    h = Header()
    h.stamp = robot_pose.header.stamp
    h.frame_id = 'confusion'
    output = PoseStamped()
    output.header = h
    output.pose.position.x = robot_pose.pose.position.x + error.position.x
    output.pose.position.y = robot_pose.pose.position.y + error.position.y
    output.pose.position.z = robot_pose.pose.position.z + error.position.z
    output.pose.orientation.x = robot_pose.pose.orientation.x + error.orientation.x
    output.pose.orientation.y = robot_pose.pose.orientation.y + error.orientation.y
    output.pose.orientation.z = robot_pose.pose.orientation.z + error.orientation.z
    output.pose.orientation.w = robot_pose.pose.orientation.w + error.orientation.w
    pub.publish(output)

def rosbot_cb(robot_pose):
    h = Header()
    h.stamp = robot_pose.header.stamp
    h.frame_id = 'confusion'
    output = PoseStamped()
    output.header = h
    output.pose.position.x = robot_pose.pose.pose.position.x + error.position.x
    output.pose.position.y = robot_pose.pose.pose.position.y + error.position.y
    output.pose.position.z = robot_pose.pose.pose.position.z + error.position.z
    output.pose.orientation.x = robot_pose.pose.pose.orientation.x + error.orientation.x
    output.pose.orientation.y = robot_pose.pose.pose.orientation.y + error.orientation.y
    output.pose.orientation.z = robot_pose.pose.pose.orientation.z + error.orientation.z
    output.pose.orientation.w = robot_pose.pose.pose.orientation.w + error.orientation.w
    pub.publish(output)

def pose_cb(diff_p):
    error.position.x = diff_p.x
    error.position.y = diff_p.y
    error.position.z = diff_p.z

def orient_cb(diff_o):
    error.orientation.x = diff_o.x
    error.orientation.y = diff_o.y
    error.orientation.z = diff_o.z
    error.orientation.w = diff_o.w

def main():
    try:
        rospy.init_node(NODE_NAME)
        #rospy.Subscriber("/mavros/local_position/pose", PoseStamped, mini2_cb, queue_size=10)
        rospy.Subscriber("/cartographer/pose", PoseStamped, mini2_cb, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, rosbot_cb, queue_size=10)
        rospy.Subscriber("/AR/confution_pose/position", Point, pose_cb, queue_size=10)
        rospy.Subscriber("/AR/confution_pose/orientation", Quaternion, orient_cb, queue_size=10)
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