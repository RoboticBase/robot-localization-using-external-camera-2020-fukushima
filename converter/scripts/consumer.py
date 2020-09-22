import os

import rospy

from handler import Handler

host = os.environ.get('AMQP_HOST', 'localhost')
port = int(os.environ.get('AMQP_PORT', '5672'))
use_tls = os.environ.get('AMQP_USE_TLS', 'False').lower() == 'true'
username = os.environ.get('AMQP_RECEIVER_USERNAME', 'ANONYMOUS')
password = os.environ.get('AMQP_RECEIVER_PASSWORD', '')
queue = os.environ.get('AMQP_RECEIVE_QUEUE', 'examples')


class Consumer(Handler):
    def __init__(self, callback):
        super().__init__(host, port, use_tls, username, password,
                         auto_accept=False)
        rospy.loginfo('init Consumer')
        self.callback = callback

    def _on_start(self, event):
        rospy.loginfo('start Consumer')
        return event.container.create_receiver(self.connection, queue)

    def on_message(self, event):
        msg = event.message.body
        rospy.loginfo('receive a message, %s', msg)
        try:
            self.callback(msg)
            self.accept(event.delivery)
            rospy.loginfo('accept this message')
        except Exception as e:
            rospy.logerr('error when calling callback, %s', e)
            self.release(event.delivery)
            rospy.loginfo('release this message')
