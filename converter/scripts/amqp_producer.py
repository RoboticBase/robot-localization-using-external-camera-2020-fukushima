#!/usr/bin/env python
import json
import signal
from datetime import datetime, timedelta, timezone
from threading import Lock

import rospy
from uoa_poc3_msgs.msg import r_state, r_info, r_navi_result

from proton.reactor import Container

from producer import Producer

SEND_DELTA_MS = 1000  # FIXME
CMD_NAME = 'open'  # FIXME


class State:
    def __init__(self, producer):
        self._producer = producer
        self._send_delta_ms = SEND_DELTA_MS
        self._prev_ms = datetime.now(timezone.utc)
        self._lock = Lock()

    def state_cb(self, state):
        rospy.loginfo('subscribe a state message, %s', state)
        now = datetime.now(timezone.utc)
        if now >= self._prev_ms + timedelta(milliseconds=self._send_delta_ms) and self._lock.acquire(False):
            self._prev_ms = now
            message = {
                'time': state.time,
                'mode': state.mode,
                'errors': [err for err in state.errors if isinstance(err, str) and len(err) > 0],
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
                'destination': {
                    'point': {
                        'x': state.destination.point.x,
                        'y': state.destination.point.y,
                        'z': state.destination.point.z,
                    },
                    'angle': {
                        'roll': state.destination.angle_optional.angle.roll,
                        'pitch': state.destination.angle_optional.angle.pitch,
                        'yaw': state.destination.angle_optional.angle.yaw,
                    } if state.destination.angle_optional.valid else None,
                },
                'covariance': list(state.covariance),
                'battery': {
                    'voltage': state.battery.voltage,
                    'current': state.battery.current_optional.current if state.battery.current_optional.valid else None,
                }
            }
            self._producer.send(json.dumps({
                'attrs': message,
            }))
            self._lock.release()


class Info:
    def __init__(self, producer):
        self._producer = producer

    def info_cb(self, info):
        rospy.loginfo('subscribe a info message, %s', info)
        message = {
            'time': info.time,
            'robot_size': {
                'robot_radius': info.robot_size.robot_radius,
                'inflation_radius': info.robot_size.inflation_radius,
                'footprint': [{'x': c.x, 'y': c.y} for c in info.robot_size.footprint],
            }
        }
        self._producer.send(json.dumps({
            'attrs': message,
        }))


class NaviResult:
    def __init__(self, producer):
        self._producer = producer
        self._cmd_name = CMD_NAME

    def navi_result_cb(self, result):
        rospy.loginfo('subscribe a navi result, %s', result)
        message = {}
        message[self._cmd_name] = {
            'time': result.time,
            'received_time': result.received_time,
            'received_cmd': result.received_cmd,
            'received_destination': {
                'point': {
                    'x': result.received_destination.point.x,
                    'y': result.received_destination.point.y,
                    'z': result.received_destination.point.z,
                },
                'angle': {
                    'roll': result.received_destination.angle_optional.angle.roll,
                    'pitch': result.received_destination.angle_optional.angle.pitch,
                    'yaw': result.received_destination.angle_optional.angle.yaw,
                } if result.received_destination.angle_optional.valid else None,
            },
            'received_costmap': {
                'resolution': result.received_costmap.resolution,
                'width': result.received_costmap.width,
                'height': result.received_costmap.height,
                'origin': {
                    'point': {
                        'x': result.received_costmap.origin.point.x,
                        'y': result.received_costmap.origin.point.y,
                        'z': result.received_costmap.origin.point.z,
                    },
                    'angle': {
                        'roll': result.received_costmap.origin.angle.roll,
                        'pitch': result.received_costmap.origin.angle.pitch,
                        'yaw': result.received_costmap.origin.angle.yaw,
                    },
                },
                'cost_value': list(result.received_costmap.cost_value),
            },
            'result': result.result,
            'errors': [err for err in result.errors if isinstance(err, str) and len(err) > 0],
        }
        self._producer.send(json.dumps({
            'cmdexe': message
        }))


def main():
    rospy.init_node('amqp_attr', anonymous=True, disable_signals=True)
    producer = Producer()

    state = State(producer)
    rospy.Subscriber('/robot_state', r_state, state.state_cb)

    info = Info(producer)
    rospy.Subscriber('/robot_info', r_info, info.info_cb)

    navi_result = NaviResult(producer)
    rospy.Subscriber('/navi_cmdexe', r_navi_result, navi_result.navi_result_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        producer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(producer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
