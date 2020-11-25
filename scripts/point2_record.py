#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
import os
import datetime
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_matrix
import math
from iot_msgs.msg import Point2

def table_title():
    buf = "count" + " ,"
    buf = buf + "robot_dir_x" + " ,"
    buf = buf + "robot_dir_y" + " ,"
    buf = buf + "robot_dir_z" + " ,"
    buf = buf + "robot_quat_x" + " ,"
    buf = buf + "robot_quat_y" + " ,"
    buf = buf + "robot_quat_z" + " ,"
    buf = buf + "robot_quat_w" + " ,"
    buf = buf + "camera_dir_x" + " ,"
    buf = buf + "camera_dir_y" + " ,"
    buf = buf + "camera_dir_z" + " ,"
    buf = buf + "camera_quat_x" + " ,"
    buf = buf + "camera_quat_y" + " ,"
    buf = buf + "camera_quat_z" + " ,"
    buf = buf + "camera_quat_w" + "\n"
    return buf

def callback(poses):
    global count
    robot_pose = poses.robot
    estimated_pose = poses.camera
    buf = str(count) + " ,"
    buf = buf + str(robot_pose.position.x) + " ,"
    buf = buf + str(robot_pose.position.y) + " ,"
    buf = buf + str(robot_pose.position.z) + " ,"
    buf = buf + str(robot_pose.orientation.x) + " ,"
    buf = buf + str(robot_pose.orientation.y) + " ,"
    buf = buf + str(robot_pose.orientation.z) + " ,"
    buf = buf + str(robot_pose.orientation.w) + " ,"
    buf = buf + str(estimated_pose.position.x) + " ,"
    buf = buf + str(estimated_pose.position.y) + " ,"
    buf = buf + str(estimated_pose.position.z) + " ,"
    buf = buf + str(estimated_pose.orientation.x) + " ,"
    buf = buf + str(estimated_pose.orientation.y) + " ,"
    buf = buf + str(estimated_pose.orientation.z) + " ,"
    buf = buf + str(estimated_pose.orientation.w) + "\n"
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
        NODE_NAME = 'point2_record'
        output_path = rospy.get_param("output_path")
        file_path = output_path + 'Point2_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '.csv'
        count = 0
        if not os.path.exists(output_path):
            os.mkdir(output_path)
        buf = table_title()
        with open(file_path, mode='a') as f:
            f.write(buf)    
        main()
    except KeyboardInterrupt:
        pass