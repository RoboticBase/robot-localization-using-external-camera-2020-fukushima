#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from eams_msgs.msg import State, Control
from std_msgs.msg import Header

def pub_start_cmd():
    h = Header()
    h.stamp.secs = 0
    h.stamp.nsecs = 0
    h.frame_id = ''
    start_cmd.header = h
    start_cmd.command = 1
    print(start_cmd)
    pub.publish(start_cmd)

def callback(state):
    global prev_status
    if(state.status == 2 and prev_status == 1):
        rospy.sleep(2.0)
        pub_start_cmd()
    prev_status = state.status


def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/command/state", State, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        prev_status = 0
        NODE_NAME = 'loop_command'
        pub = rospy.Publisher("/command/control", Control, queue_size=10)
        start_cmd = Control()
        main()
    except KeyboardInterrupt:
        pass
