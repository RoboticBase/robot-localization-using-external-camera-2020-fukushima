import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time
from tf.transformations import quaternion_matrix, quaternion_from_matrix

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

def RotationVectorToQuaternion(rvecs):
    rvecs = np.squeeze(rvecs)
    R = cv2.Rodrigues(rvecs)[0]
    R = np.vstack((R, np.zeros(R.shape[1])))
    R = np.hstack((R, np.hstack([0,0,0,1])[np.newaxis, :].T))
    q = quaternion_from_matrix(R)
    return q

def RotTranToInverseCameraMatrix(R,T):
    iC = np.vstack((R.T, np.zeros(R.T.shape[1])))
    iC = np.hstack((iC, np.hstack((np.dot(R.T, -T), np.ones(1)))[np.newaxis, :].T))
    return iC

def RotTranToCameraMatrix(R,T):
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((T, np.ones(1)))[np.newaxis, :].T))
    return C

def draw_marker(frame, ids, mtx, dist, rvecs, tvecs):
    for i in range(ids.size):
        aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.1)
        cv2.putText(frame, "X: %.1f cm" % (tvecs[0][0][0] * 100),  (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "Y: %.1f cm" % (tvecs[0][0][1] * 100),  (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "Z: %.1f cm" % (tvecs[0][0][2] * 100),  (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "R: %.1f deg" % (rvecs[0][0][0] * 180 / math.pi),  (0, 130), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "P: %.1f deg" % (rvecs[0][0][1] * 180 / math.pi),  (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "Y: %.1f deg" % (rvecs[0][0][2] * 180 / math.pi),  (0, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
    return frame

def PoseStampedtoNumpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    RotM = quaternion_matrix(Q)[:3,:3]
    CamM = RotTranToCameraMatrix(R, T)
    iCamM = RotTranToInverseCameraMatrix(R, T)
    return Tvec, Quat, RotM, CamM, iCamM


def NumpyarraytoPoseStamped(Tvec, Quat, frame_id="translate"):
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    return p
