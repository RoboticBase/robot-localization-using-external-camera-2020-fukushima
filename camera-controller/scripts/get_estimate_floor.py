#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import csv
import numpy as np
from sklearn.cluster import KMeans

NODE_NAME = 'get_estimate_floor'
input_file_path = rospy.get_param("/get_floor/input_files", "./")
ar_file = input_file_path + '/ar.csv'

def read_csv(name):
    csv_obj = csv.reader(open(name, "r"))
    l = [row for row in csv_obj]
    ary = np.array(l)
    return np.delete(ary, 0, 1)

def get_weight_position(pos_array):
    kmeans = KMeans(n_clusters=3).fit(pos_array)
    sort_centers = kmeans.cluster_centers_[kmeans.labels_[:3]]
    return sort_centers

def main():
    try:
        ar_poses_quat = read_csv(ar_file).astype(np.float32)
        sort_ar_centers = get_weight_position(ar_poses_quat)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
