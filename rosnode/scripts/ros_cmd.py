#!/usr/bin/env python
import json

import rospy
from std_msgs.msg import String


def cmd():
    rospy.init_node('ros_cmd', anonymous=True)
    pub = rospy.Publisher('/cmdexe', String, queue_size=10)

    def callback(data):
        msg = data.data
        rospy.loginfo('received msg, %s', msg)
        cmd = json.loads(msg)
        result = '{} opened successfully'.format(cmd['cmd']['open'])
        pub.publish(result)

    rospy.Subscriber('/cmd', String, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        cmd()
    except rospy.ROSInterruptException:
        pass
