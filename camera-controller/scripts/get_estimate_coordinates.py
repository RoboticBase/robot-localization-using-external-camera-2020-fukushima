#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import cv2
import datetime
import numpy as np
import math
from sklearn.cluster import KMeans
import tf
from geometry_msgs.msg import PoseStamped

NODE_NAME = 'get_estimate_coordinates'
input_file_path = rospy.get_param("/get_tmatrix/input_files", "./")
#input_file_path = '/home/rb/ARenv/Pose_2020-07-14_143324'
#output_file_path = '/home/rb/ARenv/Pose_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
pose_file = input_file_path + '/pose.csv'
ar_file = input_file_path + '/ar.csv'

def estimate_orientation(T_rotM, input):
    Rvec = quaternion_to_vector(input[3:].astype(np.float32))
    Est_vec = np.dot(T_rotM, Rvec)
    return Est_vec

def estimate_orientation2(T_camM, input):
    T = input[:3].astype(np.float32)
    Rvec = quaternion_to_vector(input[3:].astype(np.float32))
    R = vector_to_matrix(Rvec)
    C = RmatTvec_to_cameraMatrix(R, T)
    Est_cam = np.dot(T_camM, C)
    Est_vec = matrix_to_vector(Est_cam[:3, :3])
    return Est_vec

def estimate_position(T_camM, input):
    pos3 = input[:3].astype(np.float32)
    pos4 = np.hstack((pos3, np.ones(1)))
    Est_pos = np.dot(T_camM, pos4)
    return Est_pos

def quaternion_to_vector(quaternion):
    R = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    V = cv2.Rodrigues(R)[0]
    return V
def read_csv(name):
    csv_obj = csv.reader(open(name, "r"))
    l = [row for row in csv_obj]
    ary = np.array(l)
    return np.delete(ary, 0, 1)

def get_weight_position(pos_array):
    kmeans = KMeans(n_clusters=3).fit(pos_array)
    sort_centers = kmeans.cluster_centers_[kmeans.labels_[:3]]
    return sort_centers

def get_translate_pos_matrix(r_poses, ar_poses):
    y = r_poses[:, :3]
    y1 = y[1, :] - y[0, :]
    y2 = y[2, :] - y[0, :]
    y3 = np.cross(y2,y1)
    Y = np.array([y1,y2,y3]).T
    print("Y: ", Y)

    x = ar_poses[:, :3]
    x1 = x[1, :] - x[0, :]
    x2 = x[2, :] - x[0, :]
    x3 = np.cross(x2,x1)
    X = np.array([x1,x2,x3]).T
    print("X: ", X)

    A = np.dot(Y, np.linalg.inv(X))
    print("A: ", A)

    T1 = y[0, :] - np.dot(A, x[0, :])
    T2 = y[1, :] - np.dot(A, x[1, :])
    T3 = y[2, :] - np.dot(A, x[2, :])
    print("T: ", T1, T2, T3)
    Translate_CamM = RmatTvec_to_cameraMatrix(A, T1)
    print("Translate Camera Matrix: ", Translate_CamM)
    return Translate_CamM

def get_RotVector_Matrix(quaternions):
    M = np.empty([0, 3])
    for q in quaternions:
        Rvec = quaternion_to_vector(q).T
        M = np.r_[M, Rvec]
    return M.T

def get_translate_ori_matrix(r_poses, ar_poses):
    Y = get_RotVector_Matrix(r_poses[:, 3:])
    print("Y: ", Y)
    X = get_RotVector_Matrix(ar_poses[:, 3:])
    print("X: ", X)
    A = np.dot(Y, np.linalg.inv(X))
    print("A: ", A)
    print("estimated Y = AX: ", np.dot(A, X))
    return A

def RmatTvec_to_cameraMatrix(R,T):
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((T, np.ones(1)))[np.newaxis, :].T))
    return C

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat

def vector_to_matrix(vector):
    R = cv2.Rodrigues(vector)[0]
    return R

def vector_to_quarernion(vector):
    R = vector_to_matrix(vector)
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((np.zeros(R.shape[1]), np.ones(1)))[np.newaxis, :].T))
    Q = tf.transformations.quaternion_from_matrix(C)
    return Q

def matrix_to_rpy(matrix):
    threshold = 0.001
    if (abs(matrix[2,1] - 1.0) < threshold):
        roll  = math.pi / 2
        pitch = 0
        yaw   = math.atan(matrix[1,0], matrix[0,0])
    elif (abs(matrix[2,1] + 1.0) < threshold):
        roll  = - math.pi / 2
        pitch = 0
        yaw   = math.atan2(matrix[1,0], matrix[0,0])
    else:
        roll  = math.asin(matrix[2,1])
        pitch = math.atan2(-matrix[2,0], matrix[2,2])
        yaw   = math.atan2(-matrix[0,1], matrix[1,1])
    return roll, pitch, yaw

def callback(msg, args):
    inputTvec, inputQuat = PoseStamped_to_Numpyarray(msg)
    translate_ori_matrix = args[1]
    inputRvec = quaternion_to_vector(inputQuat)
    estimate_Rvec = np.dot(translate_ori_matrix, inputRvec)
    estimate_Quat = vector_to_quarernion(estimate_Rvec)

    translate_pos_matrix = args[0]
    inputRotM = tf.transformations.quaternion_matrix(inputQuat)[:3,:3]
    inputCamM = RmatTvec_to_cameraMatrix(inputRotM, inputTvec)
    estimate_Tvec = np.dot(translate_pos_matrix, inputCamM)[:3, 3]
    p = PoseStamped()
    p.header.frame_id = "estimate"
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = estimate_Tvec[0]
    p.pose.position.y = estimate_Tvec[1]
    p.pose.position.z = estimate_Tvec[2]
    p.pose.orientation.x = estimate_Quat[0]
    p.pose.orientation.y = estimate_Quat[1]
    p.pose.orientation.z = estimate_Quat[2]
    p.pose.orientation.w = estimate_Quat[3]
    pub = rospy.Publisher("/AR/estimated_pose", PoseStamped, queue_size=10)
    pub.publish(p)

def main():
    try:
        #r_coord = read_csv(pose_file)[:,:3]
        #r_quat = read_csv(pose_file)[:,3:]
        r_poses_quat = read_csv(pose_file)#np.hstack((r_coord, r_quat))
        #ar_coord = read_csv(ar_file)[:,:3]
        #ar_quat = read_csv(ar_file)[:,3:]
        ar_poses_quat = read_csv(ar_file)#np.hstack((ar_coord, ar_quat))

        sort_robot_centers = get_weight_position(r_poses_quat)
        sort_ar_centers = get_weight_position(ar_poses_quat)
        print("Centers of Robot poses: ", sort_robot_centers)
        print("Centers of AR poses: ", sort_ar_centers)
        translate_pos_matrix = get_translate_pos_matrix(sort_robot_centers, sort_ar_centers)
        translate_ori_matrix = get_translate_ori_matrix(sort_robot_centers, sort_ar_centers)

        for (ar_pose, robot_pose) in zip(ar_poses_quat, r_poses_quat):
            Rvec = quaternion_to_vector(ar_pose[3:].astype(np.float32))
            Res_vec = np.dot(translate_ori_matrix, Rvec)
            Res_mat = vector_to_matrix(Res_vec)
            Res_euler = matrix_to_rpy(Res_mat)
            robot_vec = quaternion_to_vector(robot_pose[3:].astype(np.float32))
            robot_mat = vector_to_matrix(robot_vec)
            robot_euler = matrix_to_rpy(robot_mat)
            print("estimate: ", Res_euler, ", real: ", robot_euler)
        for (ar_pose, robot_pose) in zip(ar_poses_quat, r_poses_quat):
            ar_coord = ar_pose[:3].astype(np.float32)
            ar_coord = np.hstack((ar_coord, np.ones(1)))
            Res_coord = np.dot(translate_pos_matrix, ar_coord)
            print("estimate: ", Res_coord)
            print("real: ", robot_pose[:3])

        rospy.init_node(NODE_NAME)
        rospy.Subscriber("AR/camera_pose", PoseStamped, callback, (translate_pos_matrix, translate_ori_matrix), queue_size=10)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
