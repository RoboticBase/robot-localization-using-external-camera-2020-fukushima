#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    dst = cv2.undistort(cv_image, intrinsic, distortion)
    image_message = bridge.cv2_to_imgmsg(dst, encoding="bgr8")
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
        NODE_NAME = 'calibration'
        bridge = CvBridge()
        pub = rospy.Publisher("calib_image", Image, queue_size=10)
        fs = cv2.FileStorage(rospy.get_param("calibration_path"), cv2.FILE_STORAGE_READ)
        intrinsic = fs.getNode("intrinsic").mat()
        distortion = fs.getNode("distortion").mat()
        main()
    except KeyboardInterrupt:
        pass
