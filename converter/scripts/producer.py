import rospy

from proton import Message

from handler import Handler
from utils import wrap_namespace


class Producer(Handler):
    def __init__(self):
        self._params = wrap_namespace(rospy.get_param('~'))
        super().__init__(self._params.amqp.host,
                         self._params.amqp.port,
                         self._params.amqp.use_tls,
                         self._params.amqp.username,
                         self._params.amqp.password)
        rospy.loginfo('init Producer')
        self.is_sendable = False

    def _on_start(self, event):
        rospy.loginfo('start Producer')
        return event.container.create_sender(self.connection, self._params.amqp.queue)

    def on_sendable(self, event):
        if not self.is_sendable:
            rospy.loginfo('sendable Producer')
            self.is_sendable = True

    def send(self, msg):
        if self.is_sendable:
            rospy.loginfo('send a message, %s', msg)
            self.handler.send(Message(body=msg))
