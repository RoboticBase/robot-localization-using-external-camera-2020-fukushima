#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    try:
        rospy.init_node(NODE_NAME)
        pub = rospy.Publisher("image_raw", Image, queue_size=10)
        r = rospy.Rate(10)
        bridge = CvBridge()
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FPS, 10)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        ret, frame = cap.read()
        while not rospy.is_shutdown():
            if ret == False:
                break
            ret, frame = cap.read()
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(image_message)
            r.sleep()
        cap.release()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'camera'
        main()
    except KeyboardInterrupt:
        pass
