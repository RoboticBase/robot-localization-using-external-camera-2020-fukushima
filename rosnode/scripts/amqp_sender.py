import os

import rospy

from proton import Message
from proton.handlers import MessagingHandler

AMQP_HOST = os.environ.get('AMQP_HOST', 'localhost')
AMQP_PORT = int(os.environ.get('AMQP_PORT', '5671'))
AMQP_USE_TLS = os.environ.get('AMQP_USE_TLS', 'False').lower() == 'true'
AMQP_SEND_QUEUE = os.environ.get('AMQP_SEND_QUEUE', 'example')
AMQP_SENDER_USERNAME = os.environ.get('AMQP_SENDER_USERNAME', '')
AMQP_SENDER_PASSWORD = os.environ.get('AMQP_SENDER_PASSWORD', '')


class Sender(MessagingHandler):
    def __init__(self):
        super().__init__()
        self.event = None
        self.count = 0

    def on_start(self, event):
        rospy.loginfo('Sender on_start')
        sender_options = {
            'sasl_enabled': True,
            'allowed_mechs': 'PLAIN',
            'user': AMQP_SENDER_USERNAME,
            'password': AMQP_SENDER_PASSWORD,
        }
        scheme = 'amqps' if AMQP_USE_TLS else 'amqp'
        url = '{}://{}:{}'.format(scheme, AMQP_HOST, AMQP_PORT)
        conn = event.container.connect(url, **sender_options)
        event.container.create_sender(conn, AMQP_SEND_QUEUE)

    def on_sendable(self, event):
        if self.event is None:
            rospy.loginfo('Sender on_sendable')
            self.event = event

    def send(self, msg):
        if self.event is not None:
            rospy.loginfo('send a message, %s', msg)
            self.event.sender.send(Message(body=msg))

    def shutdown(self):
        self.event.sender.close()
        self.event.connection.close()
