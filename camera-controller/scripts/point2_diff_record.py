#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
import os
import datetime
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_matrix, euler_from_matrix
import math
from iot_msgs.msg import Point2

def table_title():
    buf = "count" + " ,"
    buf = buf + "sec" + " ,"
    buf = buf + "dir_x" + " ,"
    buf = buf + "dir_y" + " ,"
    buf = buf + "dir_z" + " ,"
    buf = buf + "rad" + " ,"
    buf = buf + "deg" + "\n"
    return buf

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.position.x, msg.position.y, msg.position.z], dtype = 'float')
    Quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    TQ = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ
    
def compare_Rmatrix(R1, R2):
    return np.dot(np.linalg.inv(R1), R1)

def callback(poses):
    global count
    robot_pose = poses.robot
    estimated_pose = poses.camera
    diff_x = robot_pose.position.x - estimated_pose.position.x
    diff_y = robot_pose.position.y - estimated_pose.position.y
    diff_z = robot_pose.position.z - estimated_pose.position.z
    _, QuatR, _ = PoseStamped_to_Numpyarray(robot_pose)
    _, QuatE, _ = PoseStamped_to_Numpyarray(estimated_pose)
    Rr = quaternion_matrix(QuatR)[:3,:3]
    Re = quaternion_matrix(QuatE)[:3,:3]
    diff_rad = euler_from_matrix(compare_Rmatrix(Rr, Re))[2]
    diff_deg = diff_rad * 180 / math.pi
    sec = rospy.Time.now().to_sec()
    buf = str(count) + " ,"
    buf = buf + str(sec) + " ,"
    buf = buf + str(diff_x) + " ,"
    buf = buf + str(diff_y) + " ,"
    buf = buf + str(diff_z) + " ,"
    buf = buf + str(diff_rad) + " ,"
    buf = buf + str(diff_deg) + "\n"
    with open(diff_path, mode='a') as f:
        f.write(buf)
    print("record", count)

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
        diff_path = output_path + 'Point2_Diff_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '.csv'
        count = 0
        if not os.path.exists(output_path):
            os.mkdir(output_path)
        buf = table_title()
        with open(diff_path, mode='a') as f:
            f.write(buf)    
        main()
    except KeyboardInterrupt:
        pass