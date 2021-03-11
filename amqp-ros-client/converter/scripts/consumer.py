import rospy

from handler import Handler
from utils import wrap_namespace


class Consumer(Handler):
    def __init__(self, callback):
        self._params = wrap_namespace(rospy.get_param('~'))
        super().__init__(self._params.amqp.host,
                         self._params.amqp.port,
                         self._params.amqp.use_tls,
                         self._params.amqp.username,
                         self._params.amqp.password,
                         auto_accept=False)
        rospy.loginfo('init Consumer')
        self.callback = callback

    def _on_start(self, event):
        rospy.loginfo('start Consumer')
        return event.container.create_receiver(self.connection, self._params.amqp.queue)

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
