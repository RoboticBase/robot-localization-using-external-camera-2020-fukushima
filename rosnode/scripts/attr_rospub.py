#!/usr/bin/env python
import random

import rospy
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('chatter', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        temperature = random.uniform(0.0, 40.0)
        rospy.loginfo('temperature=%f', temperature)
        pub.publish(temperature)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
