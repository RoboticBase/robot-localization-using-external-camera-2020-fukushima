#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped
from eams_msgs.msg import State
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
import os
import datetime
from ar_func import PoseStampedToText

def Capture(im, path, name):
    if not os.path.exists(path):
        os.mkdir(path)
    cv2.imwrite(path + "/" + name + ".png", im)

def callback(msg):
    global count
    if(msg.data == "record"):
        msg1 = rospy.wait_for_message("/cartographer/pose", PoseStamped)
        msg2 = rospy.wait_for_message("/AR/camera_pose", PoseStamped)
        msg3 = rospy.wait_for_message("image_raw", Image)
        msg4 = rospy.wait_for_message("/AR/camera_image", Image)
        img = bridge.imgmsg_to_cv2(msg3, desired_encoding='bgr8')
        dst = bridge.imgmsg_to_cv2(msg4, desired_encoding='bgr8')
        buf = PoseStampedToText(count, msg1)
        with open(pose_file, mode='a') as f:
            f.write(buf)
        buf = PoseStampedToText(count, msg2)
        with open(ar_file, mode='a') as f:
            f.write(buf)
        im_name = "Img%03.f"%(count)
        Capture(img, path, im_name)
        dst_name = "Dst%03.f"%(count)
        Capture(dst, path, dst_name)
        count+=1
        print("Capturing & Logging")

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/AR/create", String, callback, queue_size=10)
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
