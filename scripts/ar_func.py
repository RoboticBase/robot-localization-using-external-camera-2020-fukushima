#!/usr/bin/python
# -*- coding: utf-8 -*-
import csv
import numpy as np
import cv2
from tf.transformations import quaternion_from_matrix

def RotationVectorToQuaternion(rvecs):
    rvecs = np.squeeze(rvecs)
    R = cv2.Rodrigues(rvecs)[0]
    R = np.vstack((R, np.zeros(R.shape[1])))
    R = np.hstack((R, np.hstack([0,0,0,1])[np.newaxis, :].T))
    q = quaternion_from_matrix(R)
    return q

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.position.x, msg.position.y, msg.position.z], dtype = 'float')
    Quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    TQ = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def compare_Rmatrix(R1, R2):
    return np.dot(np.linalg.inv(R1), R1)

def read_csv(name):
    csv_obj = csv.reader(open(name, "r"))
    l = [row for row in csv_obj]
    ary = np.array(l)
    return np.delete(ary, 0, 1)

def PoseStampedToText(count, msg):
    buf = str(count) + " ,"
    buf = buf + str(msg.pose.position.x) + " ,"
    buf = buf + str(msg.pose.position.y) + " ,"
    buf = buf + str(msg.pose.position.z) + " ,"
    buf = buf + str(msg.pose.orientation.x) + " ,"
    buf = buf + str(msg.pose.orientation.y) + " ,"
    buf = buf + str(msg.pose.orientation.z) + " ,"
    buf = buf + str(msg.pose.orientation.w) + "\n"
    return buf

