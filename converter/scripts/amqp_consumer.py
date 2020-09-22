#!/usr/bin/env python
import json
import signal

import rospy
from std_msgs.msg import String

from proton.reactor import Container

from consumer import Consumer


def main():
    rospy.init_node('amqpconsumer', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/cmd', String, queue_size=10)

    def on_message(msg):
        pub.publish(msg)
    consumer = Consumer(on_message)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        consumer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(consumer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
