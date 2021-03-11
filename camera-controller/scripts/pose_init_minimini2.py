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
from std_msgs.msg import Header, String, Float32
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Point, Quaternion

def oriToarray(ori):
    return(np.array([ori.x ,ori.y, ori.z, ori.w]))

def create_posestamped(header, pose):
    h = Header()
    h.frame_id = 'init'
    h.stamp = header.stamp
    output = PoseStamped()
    output.header = h
    output.pose.position.x = pose.position.x + init.position.x
    output.pose.position.y = pose.position.y + init.position.y
    output.pose.position.z = pose.position.z + init.position.z
    quat_in = oriToarray(pose.orientation)
    quat_err = oriToarray(init.orientation)
    quat_out = tf.transformations.quaternion_multiply(quat_in, quat_err)
    output.pose.orientation.x = quat_out[0]
    output.pose.orientation.y = quat_out[1]
    output.pose.orientation.z = quat_out[2]
    output.pose.orientation.w = quat_out[3]
    return(output)

def mini2_cb(robot_pose):
    output = create_posestamped(robot_pose.header, robot_pose.pose)
    pub.publish(output)

def rosbot2_cb(robot_pose):
    output = create_posestamped(robot_pose.header, robot_pose.pose.pose)
    pub.publish(output)

def init_cb(msg):
    if(msg.data == "zero"):
        init.position.x = 0.0
        init.position.y = 0.0
        init.position.z = 0.0
        init.orientation.x = 0.0
        init.orientation.y = 0.0
        init.orientation.z = 0.0
        init.orientation.w = 1.0
    elif(msg.data == "offset"):
        msg1 = rospy.wait_for_message("/cartographer/pose", PoseStamped)
        init.position.x = -1 * msg1.pose.position.x
        init.position.y = -1 * msg1.pose.position.y
        init.position.z = -1 * msg1.pose.position.z
        init.orientation.x = msg1.pose.orientation.x
        init.orientation.y = msg1.pose.orientation.y
        init.orientation.z = msg1.pose.orientation.z
        init.orientation.w = -1 * msg1.pose.orientation.w
def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, rosbot2_cb, queue_size=10)
        rospy.Subscriber("/cartographer/pose", PoseStamped, mini2_cb, queue_size=10)
        rospy.Subscriber("/AR/init_pose/init", String, init_cb, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'pose_confusion'
        pub = rospy.Publisher("/AR/init_pose", PoseStamped, queue_size=10)
        init = Pose()
        init.orientation.w = 1.0
        main()
    except KeyboardInterrupt:
        pass