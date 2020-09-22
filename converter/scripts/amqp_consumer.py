#!/usr/bin/env python
import json
import signal

import rospy
from geometry_msgs.msg import Point
from uoa_poc3_msgs.msg import r_command, r_pose_optional, r_angle_optional, r_angle, r_costmap, r_pose

from proton.reactor import Container

from consumer import Consumer

CMD_NAME = 'cmd'  # FIXME
ENTITY_TYPE = 'robot'  # FIXME
ENTITY_ID = 'robot01'  # FIXME
RESOLUTION = 0.05  # FIXME
WIDTH = 10  # FIXME
HEIGHT = 10  # FIXME
ORIGIN = {
    'point': {
        'x': 1.0,
        'y': 1.0,
        'z': 0.0,
    },
    'angle': {
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0,
    }
}  # FIXME


class Command:
    def __init__(self, publisher):
        self._publisher = publisher
        self._cmd_name = CMD_NAME
        self._entity_type = ENTITY_TYPE
        self._entity_id = ENTITY_ID

    def command_cb(self, msg):
        rospy.loginfo('consume a command message, %s', msg)
        try:
            message = json.loads(msg)
            body = message[self._cmd_name]
            ros_published = self._process(body)
            rospy.loginfo('processed the command, %s', ros_published)
        except (ValueError, TypeError) as e:
            rospy.logerr('invalid payload, %s', e)

    def _process(self, body):
        rospy.loginfo('process message %s', body)

        # FIXME
        body = {
            'time': '2020-09-22T05:06:07.890+00:00',
            'cmd': 'navi',
            'destination': {
                'point': {
                    'x': 0.503,
                    'y': 1.001,
                    'z': 0.0,
                },
                'angle_optional': {
                    'valid': False,
                    'angle': {
                        'roll': 0.0,
                        'pitch': 0.0,
                        'yaw': 12.3,
                    }
                }
            },
            'waypoints': [
                {
                    'x': 8.010,
                    'y': 9.010,
                    'z': 0.0,
                },
                {
                    'x': 6.100,
                    'y': 7.201,
                    'z': 0.0,
                },
                {
                    'x': 4.200,
                    'y': 5.401,
                    'z': 0.0,
                },
                {
                    'x': 2.300,
                    'y': 3.601,
                    'z': 0.0,
                },
                {
                    'x': 1.400,
                    'y': 1.801,
                    'z': 0.0,
                },
                {
                    'x': 0.503,
                    'y': 1.001,
                    'z': 0.0,
                },
            ],
        }

        command = r_command()
        command.id = self._entity_id
        command.type = self._entity_type
        command.time = body['time']
        command.cmd = body['cmd']

        angle = r_angle()
        angle.roll = body['destination']['angle_optional']['angle']['roll']
        angle.pitch = body['destination']['angle_optional']['angle']['pitch']
        angle.yaw = body['destination']['angle_optional']['angle']['yaw']
        angle_optional = r_angle_optional()
        angle_optional.valid = body['destination']['angle_optional']['valid']
        angle_optional.angle = angle
        point = Point()
        point.x = body['destination']['point']['x']
        point.y = body['destination']['point']['y']
        point.z = body['destination']['point']['z']
        destination = r_pose_optional()
        destination.point = point
        destination.angle_optional = angle_optional
        command.destination = destination

        costmap = r_costmap()
        costmap.resolution = RESOLUTION
        costmap.width = WIDTH
        costmap.height = HEIGHT
        opoint = Point()
        opoint.x = ORIGIN['point']['x']
        opoint.y = ORIGIN['point']['y']
        opoint.z = ORIGIN['point']['z']
        oangle = r_angle()
        oangle.roll = ORIGIN['angle']['roll']
        oangle.pitch = ORIGIN['angle']['pitch']
        oangle.yaw = ORIGIN['angle']['yaw']
        origin = r_pose()
        origin.point = opoint
        origin.angle = oangle
        costmap.origin = origin
        costmap.cost_value = [0, 0, 0]
        command.costmap = costmap

        self._publisher.publish(command)
        return command


def main():
    rospy.init_node('amqpconsumer', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/cmd', r_command, queue_size=10)

    command = Command(pub)
    consumer = Consumer(command.command_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        consumer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(consumer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
