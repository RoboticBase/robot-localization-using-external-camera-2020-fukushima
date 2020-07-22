#!/usr/bin/env python
import json
import signal

import rospy
from std_msgs.msg import String

from proton.reactor import Container

from receiver import Receiver


def main():
    rospy.init_node('amqpconsumer', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/cmd', String, queue_size=10)

    def on_message(msg):
        pub.publish(msg.body)
    receiver = Receiver(on_message)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        receiver.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(receiver).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
