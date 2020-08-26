#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix

def RotationVectorToQuaternion(rvecs):
    rvecs = np.squeeze(rvecs)
    R = cv2.Rodrigues(rvecs)[0]
    R = np.vstack((R, np.zeros(R.shape[1])))
    R = np.hstack((R, np.hstack([0,0,0,1])[np.newaxis, :].T))
    q = quaternion_from_matrix(R)
    return q

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

def detect_marker(frame, mtx, dist, dictionary, marker_size=0.184):
    parameters =  aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary, parameters=parameters)
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)
    return ids, rvecs, tvecs

def draw_marker(frame, ids, mtx, dist, rvecs, tvecs):
    for i in range(ids.size):
        aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.1)
        rvecs = np.squeeze(rvecs)
        R = cv2.Rodrigues(rvecs)[0]
        tvecs = np.squeeze(tvecs)
        T = tvecs[np.newaxis, :].T
        proj_matrix = np.hstack((R, T))
        euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]
        print(tvecs, euler_angle)
        cv2.putText(frame, "X: %.1f cm" % (tvecs[0] * 100),  (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "Y: %.1f cm" % (tvecs[1] * 100),  (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "Z: %.1f cm" % (tvecs[2] * 100),  (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "R: %.1f deg" % (euler_angle[0]),  (0, 130), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "P: %.1f deg" % (euler_angle[1]),  (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
        cv2.putText(frame, "Y: %.1f deg" % (euler_angle[2]),  (0, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0))
    return frame

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ids, rvecs, tvecs = detect_marker(cv_image, mtx, dist, dictionary)
    if ids is None:
        print("not found Markers")
    else:
        draw_marker(cv_image, ids, mtx, dist, rvecs, tvecs)
        pub_data(rvecs, tvecs)
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        pub.publish(image_message)
def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("image_raw", Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'ar_detector'
        bridge = CvBridge()
        pub = rospy.Publisher("/AR/camera_image", Image, queue_size=10)
        fs = cv2.FileStorage(rospy.get_param("calibration_path"), cv2.FILE_STORAGE_READ)
        mtx = fs.getNode("intrinsic").mat()
        dist = fs.getNode("distortion").mat()
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        main()
    except KeyboardInterrupt:
        pass
