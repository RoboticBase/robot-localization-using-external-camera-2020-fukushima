#!/usr/bin/env python
import json
import signal
from datetime import datetime, timedelta, timezone
from threading import Lock

import rospy
from uoa_poc3_msgs.msg import r_state

from proton.reactor import Container

from producer import Producer

SEND_DELTA_MS = 1000  # FIXME


class State:
    def __init__(self, producer):
        self._producer = producer
        self._send_delta_ms = SEND_DELTA_MS
        self._prev_ms = datetime.now(timezone.utc)
        self._lock = Lock()

    def state_cb(self, state):
        rospy.loginfo('subscribe an state message, %s', state)
        now = datetime.now(timezone.utc)
        if now >= self._prev_ms + timedelta(milliseconds=self._send_delta_ms) and self._lock.acquire(False):
            self._prev_ms = now
            message = {
                'time': now.isoformat(),
                'mode': state.mode,
                'errors': [err for err in state.errors if len(err) > 0],
                'pose': {
                    'point': {
                        'x': state.pose.point.x,
                        'y': state.pose.point.y,
                        'z': state.pose.point.z,
                    },
                    'angle': {
                        'roll': state.pose.angle.roll,
                        'pitch': state.pose.angle.pitch,
                        'yaw': state.pose.angle.yaw,
                    },
                },
                'covariance': [c for c in state.covariance],
                'battery': {
                    'voltage': state.battery.voltage,
                }
            }
            d = {
                'attrs': message,
            }
            self._producer.send(json.dumps(d))
            self._lock.release()


def main():
    rospy.init_node('amqp_attr', anonymous=True, disable_signals=True)
    producer = Producer()

    state = State(producer)
    rospy.Subscriber('/attr', r_state, state.state_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        producer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(producer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
