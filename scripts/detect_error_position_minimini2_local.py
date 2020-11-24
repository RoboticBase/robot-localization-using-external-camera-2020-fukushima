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
from geometry_msgs.msg import PoseStamped
from eams_msgs.msg import Control
from iot_msgs.msg import Point2

flg = False

def compare_Rmatrix(R1, R2):
    return np.dot(np.linalg.inv(R1), R1)

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.position.x, msg.position.y, msg.position.z], dtype = 'float')
    Quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    TQ = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def callback(poses):
    global flg
    if flg:
        return
    robot_pose = poses.robot
    estimated_pose = poses.camera
    diff_x = abs(robot_pose.position.x - estimated_pose.position.x)
    diff_y = abs(robot_pose.position.y - estimated_pose.position.y)
    _, QuatR, _ = PoseStamped_to_Numpyarray(robot_pose)
    _, QuatE, _ = PoseStamped_to_Numpyarray(estimated_pose)

    Rr = tf.transformations.quaternion_matrix(QuatR)[:3,:3]
    Re = tf.transformations.quaternion_matrix(QuatE)[:3,:3]
    diff_yaw = tf.transformations.euler_from_matrix(compare_Rmatrix(Rr, Re))[2]
    radian_threshold = degree_threshold * math.pi /180
    diff_deg = diff_yaw * 180 / math.pi
    print("COMPARE", diff_x, diff_y, diff_deg)

    if diff_x >= meter_threshold or diff_y >= meter_threshold or diff_yaw >= radian_threshold:
        print("ERROR", diff_x, diff_y, diff_deg)
        #print("robot has error pose")
        stop_order.header.stamp = rospy.Time.now()
        stop_order.header.frame_id = "stop"
        stop_order.command = 0
        pub.publish(stop_order)
        print("ROBOT STOP")
        flg = True

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/AR/integrated_pose", Point2, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'detect_error_position'
        meter_threshold = rospy.get_param("meter_threshold")
        degree_threshold = rospy.get_param("degree_threshold")
        pub = rospy.Publisher("/command/control", Control, queue_size=10)
        stop_order = Control()
        main()
    except KeyboardInterrupt:
        pass
