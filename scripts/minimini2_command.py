#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import math
from eams_msgs.msg import State, Control, Mission, Detail
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import NavSatFix

ONE_METER = 0.000008999#0.000012
halfmeter = ONE_METER / 2.0
length =    ONE_METER

#fields = "AICT" 
fields = "LICTIA"

def mini2_start():
    h = Header()
    h.stamp.secs = 0
    h.stamp.nsecs = 0
    h.frame_id = ''
    start_cmd = Control()
    start_cmd.header = h
    start_cmd.command = 1
    print(start_cmd)
    pub2.publish(start_cmd)

def mini2_waypoint(lat, lng, vel):
    detail = Detail()
    detail.command = 1
    detail.lat = lat
    detail.lng = lng
    detail.param1 = vel
    return detail

def mini2_turn():
    detail = Detail()
    detail.command = 3
    detail.lat = 0
    detail.lng = 0
    return detail

def aict_waypoint(lat ,lng, deg, vel):
    wp1 = mini2_waypoint(lat + halfmeter, lng + 0.0, vel) 
    wp2 = mini2_waypoint(lat + 0.0, lng + 0.0, vel)
    wp3 = mini2_turn() # turn
    return [wp1, wp2, wp3]

def lictia_waypoint(lat ,lng, deg, vel):
    wp1 = mini2_waypoint(lat + length, lng + 0.0, vel) 
    wp2 = mini2_waypoint(lat + length, lng + length, vel) 
    wp3 = mini2_waypoint(lat - length, lng + length, vel) 
    wp4 = mini2_waypoint(lat - length, lng - length, vel)
    wp5 = mini2_waypoint(lat + length, lng - length, vel)
    wp6 = mini2_waypoint(lat + length, lng + 0.0, vel)
    wp7 = mini2_waypoint(lat + 0.0, lng + 0.0, vel)
    wp8 = mini2_turn() # turn
    return [wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8]
    #return [wp1, wp2]


def pub_waypoint(lat ,lng, deg, vel):
    mission = Mission()
    mission.header.frame_id = "map"
    mission.header.stamp = rospy.Time.now()
    if fields == "AICT":
        mission.details = aict_waypoint(lat ,lng, deg, vel))
    if fields == "LICTIA":
        mission.details = lictia_waypoint(lat ,lng, deg, vel))
    pub1.publish(mission)
'''
def callback(state):
    global prev_status
    if(state.status == 2 and prev_status == 1):
        rospy.sleep(2.0)
        mini2_start()
    prev_status = state.status

def main():
    try:
        rospy.Subscriber("/command/state", State, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
'''
if __name__ == '__main__':
    try:
        prev_status = 0
        NODE_NAME = 'minimini2_command'
        pub1 = rospy.Publisher("/command/mission", Mission, queue_size=10)
        pub2 = rospy.Publisher("/command/control", Control, queue_size=10)
        rospy.init_node(NODE_NAME)
        msg = rospy.wait_for_message("/mavros/global_position/global", NavSatFix)
        deg = rospy.wait_for_message("/mavros/global_position/compass_hdg", Float64)
        print(msg.latitude)
        print(msg.longitude)
        print(deg.data)
        pub_waypoint(msg.latitude, msg.longitude, deg.data / 180 * math.pi, 0.03)
        rospy.sleep(5.0)
        mini2_start()
        #main()
    except KeyboardInterrupt:
        pass
