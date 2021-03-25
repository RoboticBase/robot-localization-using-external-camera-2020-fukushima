#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os

def callback(msg):
    global count
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv2.imwrite(image_path + "/%03.f"%(count)+".png", cv_image)
    count+=1
    print("Saved image")

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/image_raw", Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'save_image'
        bridge = CvBridge()
        image_path = rospy.get_param("image_path")
        count = 0
        if not os.path.exists(image_path):
            os.mkdir(image_path)
        main()
    except KeyboardInterrupt:
        pass
