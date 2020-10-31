#!/usr/bin/env python
import glob
import hashlib
import json
import os
import re
import signal
from datetime import datetime, timedelta, timezone
from threading import Lock

import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import NavSatFix, BatteryState
from eams_msgs.msg import State, ImageInfo

from proton.reactor import Container

from producer import Producer
from utils import wrap_namespace
import message_filters_py3 as message_filters


class Base:
    def __init__(self, producer):
        self._producer = producer

        alg = hashlib.sha256()
        paths = sorted([f for f in glob.glob(f'{os.path.dirname(os.path.abspath(__file__))}/../**', recursive=True)
                        if re.search('.+\\.(txt|xml|launch|py)$', f)])
        for path in paths:
            with open(path, 'rb') as f:
                while True:
                    chunk = f.read(4096 * alg.block_size)
                    if len(chunk) == 0:
                        break
                    alg.update(chunk)
        self._digest = alg.hexdigest()
        rospy.loginfo('digest=%s', self._digest)

    def _send(self, key, message):
        if 'metadata' not in message:
            message['metadata'] = {}
        message['metadata']['digest'] = self._digest
        payload = {}
        payload[key] = message
        self._producer.send(json.dumps(payload))

    def send_atts(self, message):
        self._send('attrs', message)

    def send_cmdexe(self, message):
        self._send('cmdexe', message)


class RobotState(Base):
    def __init__(self, producer):
        super().__init__(producer)
        self._params = wrap_namespace(rospy.get_param('~'))
        self._prev_ms = datetime.now(timezone.utc)
        self._lock = Lock()
        self.mode = 'init'

    def state_cb(self, position, compass, battery):
        rospy.loginfo('subscribe telemetries , position=%s, compass=%s, battery=%s', position, compass, battery)
        now = datetime.now(timezone.utc)
        if now >= self._prev_ms + timedelta(milliseconds=self._params.thresholds.send_delta_ms) and self._lock.acquire(False):
            self._prev_ms = now
            message = {
                'time': datetime.fromtimestamp(position.header.stamp.to_time(), timezone.utc).isoformat(),
                'mode': self.mode,
                'pose': {
                    'point': {
                        'latitude': position.latitude,
                        'longitude': position.longitude,
                        'altitude': position.altitude,
                    },
                    'angle': {
                        'theta': compass.data,
                    },
                },
                'accuracy': {
                    'covariance': list(position.position_covariance),
                },
                'battery': {
                    'voltage': battery.voltage,
                    'current': battery.current,
                },
                'destination': {},
                'errors': list(),
            }
            self.send_atts(message)
            self._lock.release()


    def mode_cb(self, state):
        rospy.loginfo('subscribe a state message, %s', state)
        self.mode = 'init' if state.status == 0 else \
            'navi' if state.status == 1 else \
            'standby' if state.status == 2 else \
            'suspend' if state.status == 3 else \
            'error'


class ImageInformation(Base):
    def __init__(self, producer):
        super().__init__(producer)

    def image_info_cb(self, image_info):
        rospy.loginfo('subscribe an image information message, %s', image_info)
        message = {
            'time': datetime.fromtimestamp(image_info.header.stamp.to_time(), timezone.utc).isoformat(),
            'image': {
                'time': image_info.time,
                'latitude': image_info.lat,
                'longitude': image_info.lng,
                'theta': image_info.yaw,
                'hash': image_info.hash,
                'path': image_info.path,
            },
        }
        self.send_atts(message)


class NaviResult(Base):
    def __init__(self, producer):
        super().__init__(producer)

    def navi_result_cb(self, result):
        rospy.loginfo('subscribe a navi result, %s', result)
        message = json.loads(result.data)
        self.send_cmdexe(message)


def main():
    rospy.init_node('eams_producer', anonymous=True, disable_signals=True)
    params = wrap_namespace(rospy.get_param('~'))

    producer = Producer()

    robot_state = RobotState(producer)
    position_sub = message_filters.Subscriber(params.topic.position, NavSatFix)
    compass_sub = message_filters.Subscriber(params.topic.compass, Float64)
    battery_sub = message_filters.Subscriber(params.topic.battery, BatteryState)

    slop = float(params.thresholds.slop_ms)/1000.0
    ts = message_filters.ApproximateTimeSynchronizer([position_sub, compass_sub, battery_sub], 10, slop, allow_headerless=True)
    ts.registerCallback(robot_state.state_cb)

    rospy.Subscriber(params.topic.mission_state, State, robot_state.mode_cb)

    image_info = ImageInformation(producer)
    rospy.Subscriber(params.topic.image_info, ImageInfo, image_info.image_info_cb)

    navi_result = NaviResult(producer)
    rospy.Subscriber(params.topic.navi_cmdexe, String, navi_result.navi_result_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        producer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(producer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
