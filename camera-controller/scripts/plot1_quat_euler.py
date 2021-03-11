#!/usr/bin/python
# -*- coding: utf-8 -*-
import csv
import numpy as np
import tf
import math
import cv2
pose_file = "/home/rb/ARenv/Diff_Pose_2020-08-24_180959/pose.csv"
ar_file = "/home/rb/ARenv/Diff_Pose_2020-08-24_180959/ar.csv"
pose_euler_file = "/home/rb/ARenv/Diff_Pose_2020-08-24_180959/pose_euler.csv"
ar_euler_file = "/home/rb/ARenv/Diff_Pose_2020-08-24_180959/ar_euler.csv"
diff_euler_file = "/home/rb/ARenv/Diff_Pose_2020-08-24_180959/diff_euler.csv"

def RotationMatrixToEulerAngles(R):
    threshold = 0.001
    if (abs(R[2,1] - 1.0) < threshold):
        roll  = math.pi / 2
        pitch = 0
        yaw   = math.atan(R[1,0], R[0,0])
    elif (abs(R[2,1] + 1.0) < threshold):
        roll  = - math.pi / 2
        pitch = 0
        yaw   = math.atan2(R[1,0], R[0,0])
    else:
        roll  = math.asin(R[2,1])
        pitch = math.atan2(-R[2,0], R[2,2])
        yaw   = math.atan2(-R[0,1], R[1,1])
    return roll, pitch, yaw

def quaternion_to_vector(quaternion):
    R = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    V = cv2.Rodrigues(R)[0]
    return V
def vector_to_matrix(vector):
    R = cv2.Rodrigues(vector)[0]
    return R
def quaternion_to_matrix(quaternion):
    R = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    return R
def read_csv(name):
    csv_obj = csv.reader(open(name, "r"))
    l = [row for row in csv_obj]
    ary = np.array(l)
    return np.delete(ary, 0, 1)

def main():
    r_poses_quat = read_csv(pose_file).astype(np.float32)
    r_pose = r_poses_quat[:, :3]
    r_quat = r_poses_quat[:, 3:]
    robot_R = [quaternion_to_matrix(pose.astype(np.float32)) for pose in r_quat]
    robot_euler = [RotationMatrixToEulerAngles(R) for R in robot_R]

    ar_poses_quat = read_csv(ar_file).astype(np.float32)
    ar_pose = ar_poses_quat[:, :3]
    ar_quat = ar_poses_quat[:, 3:]
    ar_R = [quaternion_to_matrix(pose.astype(np.float32)) for pose in ar_quat]
    ar_euler = [RotationMatrixToEulerAngles(R) for R in ar_R]

    ##diff_R = np.dot(np.linalg.inv(robot_R)), ar_R)
    diff_R = [np.dot(np.linalg.inv(rR), arR) for rR, arR in zip(robot_R, ar_R)]
    ## test
    #test_E = [np.dot(rR, dR) for dR, rR in zip(diff_R, robot_R)]
    #test_E = [tE - arR for tE, arR in zip(test_E, ar_R)]
    #print( test_E )
    diff_euler = [RotationMatrixToEulerAngles(R) for R in diff_R]
    diff_euler_deg = [rad * 180 / math.pi for rad in diff_euler]
    #print(diff_euler)
    '''
    with open(pose_euler_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerows(robot_euler)
    with open(ar_euler_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerows(ar_euler)

    with open(diff_euler_file, 'w') as f:
        writer = csv.writer(f)
        writer.writerows(diff_euler)
    '''
if __name__ == '__main__':
    main()
