import os

import rospy

from proton import Message

from handler import Handler

host = os.environ.get('AMQP_HOST', 'localhost')
port = int(os.environ.get('AMQP_PORT', '5672'))
use_tls = os.environ.get('AMQP_USE_TLS', 'False').lower() == 'true'
username = os.environ.get('AMQP_SENDER_USERNAME', 'ANONYMOUS')
password = os.environ.get('AMQP_SENDER_PASSWORD', '')
queue = os.environ.get('AMQP_SEND_QUEUE', 'examples')


class Producer(Handler):
    def __init__(self):
        super().__init__(host, port, use_tls, username, password)
        rospy.loginfo('init Producer')
        self.is_sendable = False

    def _on_start(self, event):
        rospy.loginfo('start Producer')
        return event.container.create_sender(self.connection, queue)

    def on_sendable(self, event):
        if not self.is_sendable:
            rospy.loginfo('sendable Producer')
            self.is_sendable = True

    def send(self, msg):
        if self.is_sendable:
            rospy.loginfo('send a message, %s', msg)
            self.handler.send(Message(body=msg))
