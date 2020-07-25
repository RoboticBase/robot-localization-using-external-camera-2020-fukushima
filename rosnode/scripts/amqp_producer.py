#!/usr/bin/env python
import json
import signal

import rospy
from std_msgs.msg import Float32, String

from proton.reactor import Container

from producer import Producer


def main():
    rospy.init_node('amqp_attr', anonymous=True, disable_signals=True)
    producer = Producer()

    count = 0
    def attr_cb(data):
        nonlocal count
        rospy.loginfo('subscribe an attr message, %f', data.data)
        count += 1
        d = {
            'attrs': {
                'count': count,
                'temperature': data.data,
            }
        }
        producer.send(json.dumps(d))
    rospy.Subscriber('/attr', Float32, attr_cb)

    def cmdexe_cb(data):
        rospy.loginfo('subscribe a cmdexe message, %s', data.data)
        d = {
            'cmdexe': {
                'open': data.data
            }
        }
        producer.send(json.dumps(d))
    rospy.Subscriber('/cmdexe', String, cmdexe_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        producer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(producer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
