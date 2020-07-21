#!/usr/bin/env python
import json
import signal

import rospy
from std_msgs.msg import Float32

from proton.reactor import Container

from sender import Sender


def main():
    rospy.init_node('amqp_attr', anonymous=True, disable_signals=True)
    sender = Sender()

    count = 0
    def callback(data):
        nonlocal count
        rospy.loginfo('subscribe a message, %f', data.data)
        count += 1
        d = {
            'attrs': {
                'count': count,
                'temperature': data.data,
            }
        }
        msg = json.dumps(d)
        sender.send(msg)
    rospy.Subscriber("/attr", Float32, callback)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        sender.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(sender).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
