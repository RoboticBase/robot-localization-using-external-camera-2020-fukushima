#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import numpy as np
import cv2
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped
import tf
## translate Orientation##
def vector_to_matrix(vector):
    R = cv2.Rodrigues(vector)[0]
    return R
def matrix_to_vector(matrix):
    return vector_to_matrix(matrix)
def RotationVectorToQuaternion(rvecs):
    rvecs = np.squeeze(rvecs)
    R = cv2.Rodrigues(rvecs)[0]
    R = np.vstack((R, np.zeros(R.shape[1])))
    R = np.hstack((R, np.hstack([0,0,0,1])[np.newaxis, :].T))
    q = quaternion_from_matrix(R)
    return q
def quaternion_to_vector(quaternion):
    R = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    V = cv2.Rodrigues(R)[0]
    return V
def vector_to_quarernion(vector):
    R = vector_to_matrix(vector)
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((np.zeros(R.shape[1]), np.ones(1)))[np.newaxis, :].T))
    Q = tf.transformations.quaternion_from_matrix(C)
    return Q
def RmatTvec_to_cameraMatrix(R,T):
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((T, np.ones(1)))[np.newaxis, :].T))
    return C

## test files##
def read_csv(name):
    csv_obj = csv.reader(open(name, "r"))
    l = [row for row in csv_obj]
    ary = np.array(l)
    return np.delete(ary, 0, 1)
def read_csv2(name):
    csv_obj = csv.reader(open(name, "r"))
    l = [row for row in csv_obj]
    ary = np.array(l)
    return ary
def PoseToText(msg):
    buf = str(msg1.position.x) + " ,"
    buf = buf + str(msg1.position.y) + " ,"
    buf = buf + str(msg1.position.z) + " ,"
    return buf

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
def PoseToText2(count, msg1, msg2):
    buf = str(count) + " ,"
    buf = buf + PoseToText(msg1)
    buf = buf + str(msg2.position.x) + " ,"
    buf = buf + str(msg2.position.y) + " ,"
    buf = buf + str(msg2.position.z) + "\n"
    return buf
def PoseStampedToText2(msg, msg_pose):
    buf = str(msg.header.stamp.nsecs) + " ,"
    buf = buf + str(msg_pose.pose.position.x) + " ,"
    buf = buf + str(msg_pose.pose.position.y) + " ,"
    buf = buf + str(msg_pose.pose.position.z) + " ,"
    buf = buf + str(msg_pose.pose.orientation.x) + " ,"
    buf = buf + str(msg_pose.pose.orientation.y) + " ,"
    buf = buf + str(msg_pose.pose.orientation.z) + " ,"
    buf = buf + str(msg_pose.pose.orientation.w) + "\n"
    return buf
def pub_data(rvecs, tvecs):
    q = RotationVectorToQuaternion(rvecs)
    tvecs = np.squeeze(tvecs)
    p = PoseStamped()
    p.header.frame_id = "camera"
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = tvecs[0]
    p.pose.position.y = tvecs[1]
    p.pose.position.z = tvecs[2]
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    pub_pose = rospy.Publisher("/AR/camera_pose", PoseStamped, queue_size=10)
    pub_pose.publish(p)

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    TQ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ
def PoseStamped_to_Numpyarray2(msg):
    Tvec = np.array([msg.position.x, msg.position.y, msg.position.z], dtype = 'float')
    Quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    TQ = np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def compare_Rmatrix(R1, R2):
    return np.dot(np.linalg.inv(R1), R1)

def latest_file(path, head_str, last_str):
    latest_time = datetime.datetime.min
    files = os.listdir(path)
    files_dir = [f for f in files if os.path.isfile(os.path.join(path, f))]
    for dir in files_dir:
        if not dir.startswith(head_str):
            continue
        if not dir.endswith(last_str):
            continue        
        name = dir.rstrip(last_str)
        dir_time = datetime.datetime.strptime(name.lstrip(head_str), '%Y-%m-%d_%H%M%S')
        if dir_time > latest_time:
            latest_time = dir_time
            latest_dir = dir
    return latest_dir

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
    
