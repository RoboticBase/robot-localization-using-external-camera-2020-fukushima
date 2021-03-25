#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import cv2
import datetime
import numpy as np
import math
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msgs import Float64
from eams_msgs.msg import Control
import os

def compare_Rmatrix(R1, R2):
    return np.dot(np.linalg.inv(R1), R1)

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    TQ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def degree_to_Numpyarray(msg):
    rad = msg.data * math.pi / 180 # degree to quat
    quat = tf.transformations.quaternion_from_euler(0, 0, rad)
    Quat = np.array([quat[0], quat[1], quat[2], quat[3], dtype = 'float')
    return Quat


def callback(robot_pose):
    estimated_pose = rospy.wait_for_message("/AR/estimated_pose", PoseStamped)
    yaw_deg = rospy.wait_for_message("/mavros/grobal_position/compass_hdg", Float64)

    diff_x = abs(robot_pose.longitude - estimated_pose.pose.position.x)
    diff_y = abs(robot_pose.latitude - estimated_pose.pose.position.y)
    #_, QuatR, _ = NavSatFix_to_Numpyarray(robot_pose)
    QuatR = degree_to_Numpyarray(yaw_deg)
    _, QuatE, _ = PoseStamped_to_Numpyarray(estimated_pose)


    Rr = tf.transformations.quaternion_matrix(QuatR)[:3,:3]
    Re = tf.transformations.quaternion_matrix(QuatE)[:3,:3]
    diff_yaw = tf.transformations.euler_from_matrix(compare_Rmatrix(Rr, Re))[2]
    radian_threshold = degree_threshold * math.pi /180
    print("COMPARE", diff_x, diff_y, diff_yaw)
    if diff_x >= meter_threshold or diff_y >= meter_threshold or diff_yaw >= radian_threshold:
        print("robot has error pose")
        stop_order.header.stamp = rospy.Time.now()
        stop_order.header.frame_id = "stop"
        stop_order.command = 0
        pub.publish(stop_order)
        print("ROBOT STOP")

def main():
    try:
        rospy.init_node(NODE_NAME)
        #rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback, queue_size=10)
        rospy.Subscriber("/mavros/grobal_position/global", NavSatFix, callback, queue_size=10)
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
