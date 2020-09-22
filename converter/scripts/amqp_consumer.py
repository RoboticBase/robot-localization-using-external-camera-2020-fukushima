#!/usr/bin/env python
import json
import signal

import rospy
from geometry_msgs.msg import Point
from uoa_poc3_msgs.msg import r_navi_command, r_pose_optional, r_angle_optional, r_angle, r_costmap, r_pose

from proton.reactor import Container
from PIL import Image, ImageDraw

from consumer import Consumer

FREE = 0
OBSTACLE = 254

CMD_NAME = 'open'  # FIXME
ENTITY_TYPE = 'robot'  # FIXME
ENTITY_ID = 'robot01'  # FIXME
RESOLUTION = 0.1  # FIXME
WIDTH = 100  # FIXME
HEIGHT = 100  # FIXME
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
RADIUS = 0.6  # FIXME


class NaviCommand:
    def __init__(self, publisher):
        self._publisher = publisher
        self._cmd_name = CMD_NAME
        self._entity_type = ENTITY_TYPE
        self._entity_id = ENTITY_ID
        self._resolution = RESOLUTION
        self._width = WIDTH
        self._height = HEIGHT
        self._origin = ORIGIN
        self._radius = RADIUS

    def command_cb(self, msg):
        rospy.loginfo('consume a navi command message, %s', msg)
        try:
            message = json.loads(msg)
            body = message['cmd'][self._cmd_name]
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
                'angle': {
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': 12.3,
                },
                # 'angle': None,
            },
            'waypoints': [
                {
                    'x': 8.010,
                    'y': 9.010,
                    'z': 0.0,
                },
                {
                    'x': 6.100,
                    'y': 8.501,
                    'z': 0.0,
                },
                {
                    'x': 4.200,
                    'y': 8.001,
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

        command = r_navi_command()
        command.id = self._entity_id
        command.type = self._entity_type
        command.time = body['time']
        command.cmd = body['cmd']

        angle = r_angle()
        angle_optional = r_angle_optional()
        if not body['destination']['angle']:
            angle.roll = 0.0
            angle.pitch = 0.0
            angle.yaw = 0.0
            angle_optional.valid = False
            angle_optional.angle = angle
        else:
            angle.roll = body['destination']['angle']['roll']
            angle.pitch = body['destination']['angle']['pitch']
            angle.yaw = body['destination']['angle']['yaw']
            angle_optional.valid = True
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
        costmap.resolution = self._resolution
        costmap.width = self._width
        costmap.height = self._height
        opoint = Point()
        opoint.x = self._origin['point']['x']
        opoint.y = self._origin['point']['y']
        opoint.z = self._origin['point']['z']
        oangle = r_angle()
        oangle.roll = self._origin['angle']['roll']
        oangle.pitch = self._origin['angle']['pitch']
        oangle.yaw = self._origin['angle']['yaw']
        origin = r_pose()
        origin.point = opoint
        origin.angle = oangle
        costmap.origin = origin
        costmap.cost_value = self._calc_cost(body['waypoints'])
        command.costmap = costmap

        self._publisher.publish(command)
        return command

    def _calc_cost(self, waypoints):
        img = Image.new('L', (self._width, self._height), color=OBSTACLE)
        draw = ImageDraw.Draw(img)
        nodes = [(int(n['x']/self._resolution), int(n['y']/self._resolution)) for n in waypoints]
        r = int(self._radius/self._resolution)
        for node in nodes:
            draw.ellipse((node[0] - r, node[1] - r, node[0] + r, node[1] + r), fill=FREE)
        draw.line(nodes, fill=FREE, width=int(r * 2))
        return list(img.getdata())


def main():
    rospy.init_node('amqpconsumer', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('/navi_cmd', r_navi_command, queue_size=10)

    naviCommand = NaviCommand(pub)
    consumer = Consumer(naviCommand.command_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        consumer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(consumer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
