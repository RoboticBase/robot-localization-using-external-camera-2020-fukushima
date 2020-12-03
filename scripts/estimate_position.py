#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
import datetime
import numpy as np
import tf
import os
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseStamped
from ar_func import read_csv

def get_weight_position(pos_array):
    kmeans = KMeans(n_clusters=3).fit(pos_array)
    sort_centers = kmeans.cluster_centers_[kmeans.labels_[:3]]
    return sort_centers

def get_translate_matrix(r_poses, ar_poses):
    y = r_poses[:, :3]
    y1 = y[1, :] - y[0, :]
    y2 = y[2, :] - y[0, :]
    y3 = np.cross(y2,y1)
    #y3 = np.cross(y1,y2)
    Y = np.array([y1,y2,y3]).T
    print("Y: ", Y)

    x = ar_poses[:, :3]
    x1 = x[1, :] - x[0, :]
    x2 = x[2, :] - x[0, :]
    x3 = np.cross(x2,x1)
    #x3 = np.cross(x1,x2)
    X = np.array([x1,x2,x3]).T
    print("X: ", X)

    A = np.dot(Y, np.linalg.inv(X))
    print("A: ", A)
    #print('############ REVERSE #######################')
    #A = np.dot(A, np.array([[-1 ,0, 0],[0 ,-1, 0],[0 ,0, 1]]))
    #print("A: ", A)

    T1 = y[0, :] - np.dot(A, x[0, :])
    T2 = y[1, :] - np.dot(A, x[1, :])
    T3 = y[2, :] - np.dot(A, x[2, :])
    print("T: ", T1, T2, T3)
    Translate_CamM = RmatTvec_to_cameraMatrix(A, T1)
    print("Translate Camera Matrix: ", Translate_CamM)
    return Translate_CamM

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
    #print(R)
    #print('############ REVERSE #######################')
    #R = np.dot(R, np.array([[1 ,0, 0],[0 ,1, 0],[0 ,0, -1]]))
    #print(R)
    C = RmatTvec_to_cameraMatrix(R, T)
    Est_C = np.dot(T_camM, C)
    Est_pos = Est_C[:3, 3]
    Est_R = Est_C[:3, :3]
    Est_vec = matrix_to_vector(Est_R)
    Est_Quat = vector_to_quarernion(Est_vec)
    return Est_Quat, Est_pos

def callback(msg, args):
    inputTvec, inputQuat, inputTQ = PoseStamped_to_Numpyarray(msg)
    #print('############ REVERSE #######################')
    #print(inputTQ)
    #inputTQ = reverse_xy2(inputTQ)
    #print(inputTQ)
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
    pub = rospy.Publisher("/AR/estimated_pose", PoseStamped, queue_size=10)
    pub.publish(p)

def reverse_xy(pose_quat):
    pose_quat = pose_quat * np.array([[-1, -1, 1, -1, -1, 1, -1],[-1, -1, 1, -1, -1, 1, -1],[-1, -1, 1, -1, -1, 1, -1]])
    return pose_quat
def reverse_xy2(pose_quat):
    pose_quat = pose_quat * np.array([-1, -1, 1, -1, -1, 1, -1])
    return pose_quat


def latest_dir(path, head_str):
    latest_time = datetime.datetime.min
    files = os.listdir(path)
    files_dir = [f for f in files if os.path.isdir(os.path.join(path, f))]
    for dir in files_dir:
        if not dir.startswith(head_str):
            continue
        dir_time = datetime.datetime.strptime(dir.lstrip(head_str), '%Y-%m-%d_%H%M%S')
        if dir_time > latest_time:
            latest_time = dir_time
            latest_dir = dir
    return latest_dir

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("AR/camera_pose", PoseStamped, callback, translate_matrix, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'get_estimate_coordinates'
        file_path = rospy.get_param("file_path")
        latest_time = latest_dir(file_path, "Pose_")
        print("READED position from ", latest_time)
        pose_file = file_path + latest_time + '/pose.csv'
        ar_file = file_path + latest_time + '/ar.csv'
        r_poses_quat = read_csv(pose_file).astype(np.float32)
        ar_poses_quat = read_csv(ar_file).astype(np.float32)
        sort_robot_centers = get_weight_position(r_poses_quat)
        sort_ar_centers = get_weight_position(ar_poses_quat)
        #print('############ REVERSE #######################')
        #sort_ar_centers = reverse_xy(sort_ar_centers)
        print("Centers of Robot poses: ", sort_robot_centers)
        print("Centers of AR poses: ", sort_ar_centers)
        translate_matrix = get_translate_matrix(sort_robot_centers, sort_ar_centers)
        for (ar_pose, robot_pose) in zip(ar_poses_quat, r_poses_quat):
            Est_Quat, Est_pos = estimate(translate_matrix, ar_pose)
            robot_vec = quaternion_to_vector(robot_pose[3:].astype(np.float32))
            robot_mat = vector_to_matrix(robot_vec)
            #print("estimate[m]: ", Est_pos, ", real[m]: ", robot_pose[:3].astype(np.float32))
            print("diff_pose: ", Est_pos - robot_pose[:3].astype(np.float32))
            print("estimate[quat]: ", Est_Quat, ", real[quat]: ", robot_pose[3:])
            #print("estimate[rad]: ", Est_euler, ", real[rad]: ", robot_euler)
            #print("estimate[deg]: ", [deg * 180 / math.pi for deg in Est_euler], ", real[deg]: ", [deg * 180 / math.pi for deg in robot_euler])
        main()
    except KeyboardInterrupt:
        pass
