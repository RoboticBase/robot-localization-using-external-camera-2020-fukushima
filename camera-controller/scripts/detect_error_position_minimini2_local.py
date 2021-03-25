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
from ar_func import PoseStamped_to_Numpyarray2, compare_Rmatrix
from std_msgs.msg import String
flg = False
def stop_command():
    stop_order = Control()
    stop_order.header.stamp = rospy.Time.now()
    stop_order.header.frame_id = "stop"
    stop_order.command = 0
    return stop_order

def alert():
    alert = String()
    alert.data = "error_position_alert"
    return alert

def pub_stop():
    pub.publish(stop_command())

def callback(poses):
    global flg
    if flg:
        pub_stop()
        return
    robot_pose = poses.robot
    estimated_pose = poses.camera
    diff_x = abs(robot_pose.position.x - estimated_pose.position.x)
    diff_y = abs(robot_pose.position.y - estimated_pose.position.y)
    _, QuatR, _ = PoseStamped_to_Numpyarray2(robot_pose)
    _, QuatE, _ = PoseStamped_to_Numpyarray2(estimated_pose)
    Rr = tf.transformations.quaternion_matrix(QuatR)[:3,:3]
    Re = tf.transformations.quaternion_matrix(QuatE)[:3,:3]
    diff_yaw = tf.transformations.euler_from_matrix(compare_Rmatrix(Rr, Re))[2]
    radian_threshold = degree_threshold * math.pi /180
    diff_deg = diff_yaw * 180 / math.pi
    print(diff_yaw)
    if diff_x >= meter_threshold or diff_y >= meter_threshold or abs(diff_yaw) >= radian_threshold:
        print("ERROR", diff_x, diff_y, diff_deg)
        pub_stop()
        pub2.publish(alert())
        flg = True
    else:
        print("COMPARE", diff_x, diff_y, diff_deg)

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
        pub2 = rospy.Publisher("/alert/play", String, queue_size=10)
        main()

    except KeyboardInterrupt:
        pass
