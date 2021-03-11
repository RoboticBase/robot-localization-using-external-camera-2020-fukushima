#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import datetime
import time
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

NODE_NAME = 'record_result'
file_path = rospy.get_param("file_path")
path = file_path + '/Estimate_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
estimate_file = path + '/estimate.csv'
result_file = path + '/robot.csv'
count = 0

def PoseStampedToTextDate(date, msg):
    buf = str(date) + " ,"
    buf = buf + str(msg.pose.position.x) + " ,"
    buf = buf + str(msg.pose.position.y) + " ,"
    buf = buf + str(msg.pose.position.z) + " ,"
    buf = buf + str(msg.pose.orientation.x) + " ,"
    buf = buf + str(msg.pose.orientation.y) + " ,"
    buf = buf + str(msg.pose.orientation.z) + " ,"
    buf = buf + str(msg.pose.orientation.w) + "\n"
    return buf
def PoseWithCovarianceStampedToTextDate(date, msg):
    buf = str(date) + " ,"
    buf = buf + str(msg.pose.pose.position.x) + " ,"
    buf = buf + str(msg.pose.pose.position.y) + " ,"
    buf = buf + str(msg.pose.pose.position.z) + " ,"
    buf = buf + str(msg.pose.pose.orientation.x) + " ,"
    buf = buf + str(msg.pose.pose.orientation.y) + " ,"
    buf = buf + str(msg.pose.pose.orientation.z) + " ,"
    buf = buf + str(msg.pose.pose.orientation.w) + "\n"
    return buf
def callback(msg):
    print("RESULT RECORDED")
    global path
    global estimate_file
    global result_file
    time = datetime.datetime.now().strftime('%H%M%S')


    estimate = rospy.wait_for_message("/AR/camera_pose", PoseStamped)
    estimate = PoseStampedToTextDate(time, estimate)
    robot = PoseWithCovarianceStampedToTextDate(time, msg)
    #estimate = PoseStampedToTextDate(time, msg)
    #robot_val = PoseWithCovarianceStampedToTextDate(time, robot)
    with open(estimate_file, mode='a') as f:
        f.write(estimate)
    with open(result_file, mode='a') as f:
        f.write(robot)
def main():
    try:
        rospy.init_node(NODE_NAME)
        #robot = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        #rospy.Subscriber("/AR/camera_pose", PoseStamped, callback, queue_size=10)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback, queue_size=10)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    if not os.path.exists(path):
        os.mkdir(path)
    if not os.path.isfile(estimate_file):
        f = open(estimate_file,'a')
        f.close()
    if not os.path.isfile(result_file):
        f = open(result_file,'a')
        f.close()
    main()
