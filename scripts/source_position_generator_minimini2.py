#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped
from rpl.msg import State
from actionlib_msgs.msg import GoalStatusArray
import os
import datetime

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

def report():
    global count
    #msg1 = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    msg1 = rospy.wait_for_message("/mavros/local_position/pose", PoseStamped)
    msg2 = rospy.wait_for_message("/image_raw", Image)
    msg3 = rospy.wait_for_message("/AR/camera_pose", PoseStamped)
    buf = PoseStampedToText(count, msg1)
    with open(pose_file, mode='a') as f:
        f.write(buf)
    cv_image = bridge.imgmsg_to_cv2(msg2, desired_encoding='bgr8')
    cv2.imwrite(path + "/%03.f"%(count)+".png", cv_image)
    buf = PoseStampedToText(count, msg3)
    with open(ar_file, mode='a') as f:
        f.write(buf)
    count+=1
    print("RECORDED Source position")

def callback(msg):
    global prev_status
    #if not msg.status: return
    if(msg.status == 2 and prev_status == 1):
        report()
    prev_status = msg.status

def main():
    try:
        rospy.init_node(NODE_NAME)
        #rospy.Subscriber("/move_base/status", GoalStatusArray, callback, queue_size=10)
        rospy.Subscriber("/mission/state", State, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'source_position_generator'
        bridge = CvBridge()
        file_path = rospy.get_param("file_path")
        prev_status = 0
        path = file_path + 'Pose_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '/'
        pose_file = path + 'pose.csv'
        ar_file = path + 'ar.csv'
        count = 0
        if not os.path.exists(path):
            os.mkdir(path)
        main()
    except KeyboardInterrupt:
        pass
