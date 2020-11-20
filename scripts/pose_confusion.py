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
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Point, Quaternion

def oriToarray(ori):
    return(np.array([ori.x ,ori.y, ori.z, ori.w]))

def create_posestamped(header, pose):
    h = Header()
    h.frame_id = 'confusion'
    h.stamp = header.stamp
    output = PoseStamped()
    output.header = h
    output.pose.position.x = pose.position.x + error.position.x
    output.pose.position.y = pose.position.y + error.position.y
    output.pose.position.z = pose.position.z + error.position.z
    quat_in = oriToarray(pose.orientation)
    quat_err = oriToarray(error.orientation)
    quat_out = tf.transformations.quaternion_multiply(quat_in, quat_err)
    output.pose.orientation.x = quat_out[0]
    output.pose.orientation.y = quat_out[1]
    output.pose.orientation.z = quat_out[2]
    output.pose.orientation.w = quat_out[3]
    return(output)

def mini2_cb(robot_pose):
    output = create_posestamped(robot_pose.header, robot_pose.pose)
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

def degree_cb(deg):
    rad = deg.data / 180 * math.pi
    quat = tf.transformations.quaternion_from_euler(0,0,rad)
    error.orientation.x = quat[0]
    error.orientation.y = quat[1]
    error.orientation.z = quat[2]
    error.orientation.w = quat[3]

def main():
    try:
        rospy.init_node(NODE_NAME)
        #rospy.Subscriber("/mavros/local_position/pose", PoseStamped, mini2_cb, queue_size=10)
        rospy.Subscriber("/AR/init_pose", PoseStamped, mini2_cb, queue_size=10)
        rospy.Subscriber("/AR/confution_pose/position", Point, pose_cb, queue_size=10)
        rospy.Subscriber("/AR/confution_pose/degree", Float32, degree_cb, queue_size=10)
        rospy.Subscriber("/AR/confution_pose/orientation", Quaternion, orient_cb, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'pose_confusion'
        pub = rospy.Publisher("/AR/confution_pose", PoseStamped, queue_size=10)
        error = Pose()
        error.orientation.w = 1.0
        main()
    except KeyboardInterrupt:
        pass