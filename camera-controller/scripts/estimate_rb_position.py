#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
import csv
import datetime
import numpy as np
import tf
import os
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseStamped
from ar_func import read_csv

def quaternion_to_vector(quaternion):
    R = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    V = cv2.Rodrigues(R)[0]
    return V

def RmatTvec_to_cameraMatrix(R,T):
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((T, np.ones(1)))[np.newaxis, :].T))
    return C

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    TQ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def vector_to_matrix(vector):
    R = cv2.Rodrigues(vector)[0]
    return R
def matrix_to_vector(matrix):
    return vector_to_matrix(matrix)

def vector_to_quarernion(vector):
    R = vector_to_matrix(vector)
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((np.zeros(R.shape[1]), np.ones(1)))[np.newaxis, :].T))
    Q = tf.transformations.quaternion_from_matrix(C)
    return Q

def estimate(T_camM, input):
    T = input[:3].astype(np.float32)
    Rvec = quaternion_to_vector(input[3:].astype(np.float32))
    R = vector_to_matrix(Rvec)
    C = RmatTvec_to_cameraMatrix(R, T)
    Est_C = np.dot(T_camM, C)
    Est_pos = Est_C[:3, 3]
    Est_R = Est_C[:3, :3]
    Est_vec = matrix_to_vector(Est_R)
    Est_Quat = vector_to_quarernion(Est_vec)
    return Est_Quat, Est_pos

def callback(msg, args):
    inputTvec, inputQuat, inputTQ = PoseStamped_to_Numpyarray(msg)
    translate_matrix = args
    Est_Quat, Est_pos = estimate(translate_matrix, inputTQ)
    p = PoseStamped()
    p.header.frame_id = "estimate"
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = Est_pos[0]
    p.pose.position.y = Est_pos[1]
    p.pose.position.z = Est_pos[2]
    p.pose.orientation.x = Est_Quat[0]
    p.pose.orientation.y = Est_Quat[1]
    p.pose.orientation.z = Est_Quat[2]
    p.pose.orientation.w = Est_Quat[3]
    pub.publish(p)

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/cartographer/pose", PoseStamped, callback, translate_matrix, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'estimate_rb_position'
        file_path = rospy.get_param("matrix_path")
        pose_file = file_path + "tmatrix_f_rb.csv"
        print("READED tMatrix from ", pose_file)
        csv_obj = csv.reader(open(pose_file, "r"))
        l = [row for row in csv_obj]
        tmatrix = np.array(l)
        translate_matrix = tmatrix.astype(np.float32)
        pub = rospy.Publisher("/RB/estimated_pose", PoseStamped, queue_size=10)
        main()

    except KeyboardInterrupt:
        pass
