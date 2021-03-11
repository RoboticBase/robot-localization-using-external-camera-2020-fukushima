#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix

import Functions

NODE_NAME = 'translator'

def callback(msg, args):
    _, _, _, C_CamM, _ = Functions.PoseStampedtoNumpyarray(msg)
    _, _, _, T_CamM, _ = Functions.PoseStampedtoNumpyarray(args)

    R_CamM = np.dot(T_CamM, C_CamM)
    R_RotM = R_CamM[:3,:3]
    R_RotM = np.vstack((R_RotM, np.zeros(R_RotM.shape[1])))
    R_RotM = np.hstack((R_RotM, np.hstack([0,0,0,1])[np.newaxis, :].T))
    R_Quat = quaternion_from_matrix(R_RotM)

    p = PoseStamped()
    p.header.frame_id = "translate"
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = R_CamM[0,3]
    p.pose.position.y = R_CamM[1,3]
    p.pose.position.z = R_CamM[2,3]
    p.pose.orientation.x = R_Quat[0]
    p.pose.orientation.y = R_Quat[1]
    p.pose.orientation.z = R_Quat[2]
    p.pose.orientation.w = R_Quat[3]
    pub = rospy.Publisher("/AR/robot_pose", PoseStamped, queue_size=10)
    pub.publish(p)

def main():
    try:
        rospy.init_node(NODE_NAME)
        TranslateMatrix=rospy.wait_for_message("/AR/translate_matrix", PoseStamped)
        rospy.Subscriber("AR/camera_pose", PoseStamped, callback, TranslateMatrix, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
