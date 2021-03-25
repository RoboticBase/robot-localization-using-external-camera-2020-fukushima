#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
import csv
import datetime
import numpy as np
import tf
import os
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseStamped
from ar_func import read_csv

def latest_file(path, head_str, last_str):
    latest_time = datetime.datetime.min
    files = os.listdir(path)
    files_dir = [f for f in files if os.path.isfile(os.path.join(path, f))]
    for dir in files_dir:
        if not dir.startswith(head_str):
            continue
        if not dir.endswith(last_str):
            continue        
        name = dir.rstrip(last_str)
        dir_time = datetime.datetime.strptime(name.lstrip(head_str), '%Y-%m-%d_%H%M%S')
        if dir_time > latest_time:
            latest_time = dir_time
            latest_dir = dir
    return latest_dir

def main():
    try:
        pass
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'adjust_matrix'
        tmatrix_path = rospy.get_param("matrix_path")
        record_path = rospy.get_param("record_path")
        adjust_path = rospy.get_param("adjust_path")
        latest_time = latest_file(tmatrix_path, "TranslateM_", ".csv")
        tmatrix_file = tmatrix_path + latest_time
        print(tmatrix_file)
        latest_time = latest_file(record_path, "Point2_Diff_", ".csv")
        record_file = record_path + latest_time
        print(record_file)

    except KeyboardInterrupt:
        pass