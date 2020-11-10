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
from eams_msgs.msg import State, Mission, Control
from iot_msgs.msg import Point2
from proton.reactor import Container

from producer import Producer
from utils import wrap_namespace
import message_filters_py3 as message_filters


class Base:
    def __init__(self, producer):
        self._producer = producer

    def _send(self, key, message):
        if 'metadata' not in message:
            message['metadata'] = {}
        payload = {}
        payload[key] = message
        self._producer.send(json.dumps(payload))

    def send_atts(self, message):
        self._send('attrs', message)

    def send_cmdexe(self, message):
        self._send('cmdexe', message)

    def send_cmd(self, message):
        self._send('cmd', message)

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

    def poses_cb(self, poses):
        rospy.loginfo('subscribe poses message, %s', poses)
        message = {
            'time': datetime.fromtimestamp(poses.header.stamp.to_time(), timezone.utc).isoformat(),
            'pose_camera': {
                'point': {
                    'x': poses.camera.position.x,
                    'y': poses.camera.position.y,
                    'z': poses.camera.position.z,
                },
                'orientation': {
                    'x': poses.camera.orientation.x,
                    'y': poses.camera.orientation.y,
                    'z': poses.camera.orientation.z,
                    'w': poses.camera.orientation.w
                }
            },
            'pose_robot': {
                'point': {
                    'x': poses.robot.position.x,
                    'y': poses.robot.position.y,
                    'z': poses.robot.position.z,
                },
                'orientation': {
                    'x': poses.robot.orientation.x,
                    'y': poses.robot.orientation.y,
                    'z': poses.robot.orientation.z,
                    'w': poses.robot.orientation.w
                }
            }
        }
        self.send_atts(message)

    def mode_cb(self, state):
        rospy.loginfo('subscribe a state message, %s', state)
        self.mode = 'init' if state.status == 0 else \
            'navi' if state.status == 1 else \
            'standby' if state.status == 2 else \
            'suspend' if state.status == 3 else \
            'error'

class RobotCommand(Base):
    def __init__(self, producer):
        super().__init__(producer)

    def control_cb(self, control):
        rospy.loginfo('subscribe a control message, %s', control)
        if control.command == 1:
            cmd = 'start'
        elif control.command == 0:
            cmd = 'stop'
        elif control.command == 2:
            cmd = 'suspend'
        else:
            rospy.logerr('invalid command {}'.format(cmd))
            return
        message = {
            'naviCmd': {
                'time': datetime.fromtimestamp(control.header.stamp.to_time(), timezone.utc).isoformat(),
                'command': cmd
            }
        }
        self.send_cmd(message)

    def mission_cb(self, mission):
        rospy.loginfo('subscribe a mission message, %s', mission)
        pass

def main():
    rospy.init_node('iot_producer', anonymous=True, disable_signals=True)
    params = wrap_namespace(rospy.get_param('~'))

    producer = Producer()

    robot_state = RobotState(producer)
    rospy.Subscriber(params.topic.mission_state, State, robot_state.mode_cb)

    robot_command = RobotCommand(producer)
    rospy.Subscriber(params.topic.control_cmd, Control, robot_command.control_cb)
    rospy.Subscriber(params.topic.mission_cmd, Mission, robot_command.mission_cb)

    rospy.Subscriber("/AR/integrated_pose", Point2, robot_state.poses_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        producer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(producer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
