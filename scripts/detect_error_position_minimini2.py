#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import cv2
import datetime
import numpy as np
import math
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import os

def compare_Rmatrix(R1, R2):
    return np.dot(np.linalg.inv(R1), R1)

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    TQ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def callback(robot_pose):
    estimated_pose = rospy.wait_for_message("/AR/estimated_pose", PoseStamped)
    diff_x = abs(robot_pose.pose.position.x - estimated_pose.pose.position.x)
    diff_y = abs(robot_pose.pose.position.y - estimated_pose.pose.position.y)
    _, QuatR, _ = PoseStamped_to_Numpyarray(robot_pose)
    _, QuatE, _ = PoseStamped_to_Numpyarray(estimated_pose)

    Rr = tf.transformations.quaternion_matrix(QuatR)[:3,:3]
    Re = tf.transformations.quaternion_matrix(QuatE)[:3,:3]
    diff_yaw = tf.transformations.euler_from_matrix(compare_Rmatrix(Rr, Re))[2]
    radian_threshold = degree_threshold * math.pi /180
    print("COMPARE", diff_x, diff_y, diff_yaw)
    if diff_x >= meter_threshold or diff_y >= meter_threshold or diff_yaw >= radian_threshold:
        print("robot has error pose")

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'detect_error_position'
        meter_threshold = rospy.get_param("meter_threshold")
        degree_threshold = rospy.get_param("degree_threshold")
        main()
    except KeyboardInterrupt:
        pass
