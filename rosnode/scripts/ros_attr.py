#!/usr/bin/env python
import random

import rospy
from std_msgs.msg import Float32


def attr():
    rospy.init_node('ros_attr', anonymous=True)
    pub = rospy.Publisher('/attr', Float32, queue_size=10)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        temperature = random.uniform(0.0, 40.0)
        rospy.loginfo('temperature=%f', temperature)
        pub.publish(temperature)
        r.sleep()


if __name__ == '__main__':
    try:
        attr()
    except rospy.ROSInterruptException:
        pass
