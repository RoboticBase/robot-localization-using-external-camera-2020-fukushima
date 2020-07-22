import os

import rospy

from proton import Message
from proton.handlers import MessagingHandler

AMQP_HOST = os.environ.get('AMQP_HOST', 'localhost')
AMQP_PORT = int(os.environ.get('AMQP_PORT', '5671'))
AMQP_USE_TLS = os.environ.get('AMQP_USE_TLS', 'False').lower() == 'true'
AMQP_RECEIVE_QUEUE = os.environ.get('AMQP_RECEIVE_QUEUE', 'example')
AMQP_RECEIVER_USERNAME = os.environ.get('AMQP_RECEIVER_USERNAME', '')
AMQP_RECEIVER_PASSWORD = os.environ.get('AMQP_RECEIVER_PASSWORD', '')


class Receiver(MessagingHandler):
    def __init__(self, callback):
        super().__init__()
        self.connection = None
        self.receiver = None
        self.callback = callback

    def on_start(self, event):
        rospy.loginfo('Receiver on_start')
        receiver_options = {
            'sasl_enabled': True,
            'allowed_mechs': 'PLAIN',
            'user': AMQP_RECEIVER_USERNAME,
            'password': AMQP_RECEIVER_PASSWORD,
        }
        scheme = 'amqps' if AMQP_USE_TLS else 'amqp'
        url = '{}://{}:{}'.format(scheme, AMQP_HOST, AMQP_PORT)
        self.connection = event.container.connect(url, **receiver_options)
        event.container.create_receiver(self.connection, AMQP_RECEIVE_QUEUE)

    def on_message(self, event):
        if self.receiver is None:
            self.receiver = event.receiver
        rospy.loginfo('receive a message,  %s', event.message)
        self.callback(event.message)

    def shutdown(self):
        if self.receiver is not None:
            self.receiver.close()
        if self.connection is not None:
            self.connection.close()
