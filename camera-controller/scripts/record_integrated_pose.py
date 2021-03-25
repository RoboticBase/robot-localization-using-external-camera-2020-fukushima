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
from ar_func import PoseStamped_to_Numpyarray, compare_Rmatrix
from ar_func import PoseToText
file_path = '/home/rb/camera_ws/src/rpl/config/log_image_robot/result.csv'


flg = False
def callback(poses):
    global count
    robot_pose = poses.robot
    estimated_pose = poses.camera
    buf = PoseToText(count, robot_pose, estimated_pose)
    print(buf)
    with open(file_path, mode='a') as f:
        f.write(buf)
    count+=1
def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/AR/integrated_pose", Point2, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'record_integrated_pose'
        count = 0
        main()
    except KeyboardInterrupt:
        pass
