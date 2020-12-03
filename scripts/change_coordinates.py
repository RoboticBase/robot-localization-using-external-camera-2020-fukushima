#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_matrix

import Functions

NODE_NAME = 'change_coordinates'
fps = 100.
delay = 1/fps*0.5

def main():
    try:
        rospy.init_node(NODE_NAME)
        msg1 = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        msg2 = rospy.wait_for_message("/AR/camera_pose", PoseStamped)

        _,　_, _, R_CamM, _ = PoseStampedtoNumpyarray(msg1.pose)
        _,　_, _, _, C_iCamM = PoseStampedtoNumpyarray(msg2)
        Translate_CamM = np.dot(R_CamM, C_iCamM)
        Translate_RotM = Translate_CamM[:3,:3]
        Translate_RotM = np.vstack((Translate_RotM, np.zeros(Translate_RotM.shape[1])))
        Translate_RotM = np.hstack((Translate_RotM, np.hstack([0,0,0,1])[np.newaxis, :].T))
        Translate_Quat = quaternion_from_matrix(Translate_RotM)

#        p = NumpyarraytoPoseStamped()
        p = PoseStamped()
        p.header.frame_id = "translate"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = Translate_CamM[0,3]
        p.pose.position.y = Translate_CamM[1,3]
        p.pose.position.z = Translate_CamM[2,3]
        p.pose.orientation.x = Translate_Quat[0]
        p.pose.orientation.y = Translate_Quat[1]
        p.pose.orientation.z = Translate_Quat[2]
        p.pose.orientation.w = Translate_Quat[3]
        pub = rospy.Publisher("/AR/translate_matrix", PoseStamped, queue_size=10)
        time.sleep(10)
        pub.publish(p)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
