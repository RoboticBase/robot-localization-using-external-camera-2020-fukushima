#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import datetime
import os
NODE_NAME = 'comparison'
file_path = rospy.get_param("file_path")

path = file_path + '/Diff_Pose_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
pose_file = path + '/pose.csv'
ar_file = path + '/ar.csv'
diff_file = path + '/diff.csv'

def quaternion_to_vector(quaternion):
    R = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    V = cv2.Rodrigues(R)[0]
    return V

def PoseStampedToText(msg, msg_pose):
    buf = str(msg.header.stamp.nsecs) + " ,"
    buf = buf + str(msg_pose.pose.position.x) + " ,"
    buf = buf + str(msg_pose.pose.position.y) + " ,"
    buf = buf + str(msg_pose.pose.position.z) + " ,"
    buf = buf + str(msg_pose.pose.orientation.x) + " ,"
    buf = buf + str(msg_pose.pose.orientation.y) + " ,"
    buf = buf + str(msg_pose.pose.orientation.z) + " ,"
    buf = buf + str(msg_pose.pose.orientation.w) + "\n"
    return buf

def Diff_PoseStampedToText(msg1, msg2):
    buf = str(msg1.header.stamp.nsecs) + " ,"
    buf = buf + str(abs(msg1.pose.pose.position.x - msg2.pose.position.x)) + " ,"
    buf = buf + str(abs(msg1.pose.pose.position.y - msg2.pose.position.y)) + " ,"
    buf = buf + str(abs(msg1.pose.pose.position.z - msg2.pose.position.z)) + " ,"
    buf = buf + str(abs(msg1.pose.pose.orientation.x - msg2.pose.orientation.x)) + " ,"
    buf = buf + str(abs(msg1.pose.pose.orientation.y - msg2.pose.orientation.y)) + " ,"
    buf = buf + str(abs(msg1.pose.pose.orientation.z - msg2.pose.orientation.z)) + " ,"
    buf = buf + str(abs(msg1.pose.pose.orientation.w - msg2.pose.orientation.w)) + "\n"
    return buf

def callback(msg):
    estimate = rospy.wait_for_message("/AR/estimated_pose", PoseStamped)
    if(msg.header.stamp.nsecs == estimate.header.stamp.nsecs):
        buf = PoseStampedToText(msg, msg.pose)
        with open(pose_file, mode='a') as f:
            f.write(buf)
        buf = PoseStampedToText(estimate, estimate)
        with open(ar_file, mode='a') as f:
            f.write(buf)
        buf = Diff_PoseStampedToText(msg, estimate)
        with open(diff_file, mode='a') as f:
            f.write(buf)

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    if not os.path.exists(path):
        os.mkdir(path)
    if not os.path.isfile(pose_file):
        f = open(pose_file,'a')
        f.close()
    if not os.path.isfile(ar_file):
        f = open(ar_file,'a')
        f.close()
    if not os.path.isfile(ar_file):
        f = open(diff_file,'a')
        f.close()
    main()
