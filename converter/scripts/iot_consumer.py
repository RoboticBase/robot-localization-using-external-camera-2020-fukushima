#!/usr/bin/env python
import json
import signal
from datetime import datetime, timezone

import rospy

from std_msgs.msg import Header, String
from eams_msgs.msg import Control, Mission, Detail

from proton.reactor import Container

from consumer import Consumer
from utils import wrap_namespace


class Dispatcher:
    def __init__(self, naviCommand):
        self._params = wrap_namespace(rospy.get_param('~'))
        self._naviCommand = naviCommand

    def dispatch_cb(self, msg):
        rospy.loginfo('consume a command message, %s', msg)
        try:
            message = json.loads(msg)
            if 'cmd' not in message:
                rospy.logerr('invalid payload')
                return
            if self._params.rb.navi_cmd_name in message['cmd']:
                body = message['cmd'][self._params.rb.navi_cmd_name]
                ros_published_control, ros_published_mission = self._naviCommand.process(body)
                rospy.loginfo('processed the navi command, control=%s, mission=%s', ros_published_control, ros_published_mission)
            else:
                rospy.logerr('unknown command')
        except (ValueError, TypeError) as e:
            rospy.logerr('invalid payload, %s', e)


class NaviCommand:
    def __init__(self, control_pub, mission_pub, cmdexe_pub):
        self._params = wrap_namespace(rospy.get_param('~'))
        self._control_pub = control_pub
        self._mission_pub = mission_pub
        self._cmdexe_pub = cmdexe_pub
        self._mission_wait_ms = float(self._params.thresholds.mission_wait_ms)/1000.0
        self._waypoint_wait_ms = float(self._params.thresholds.waypoint_wait_ms)/1000.0

    def process(self, body):
        rospy.loginfo('process a navi message %s', body)

        control, mission = self._make_control_and_mission(body)
        if mission is not None:
            self._mission_pub.publish(mission)
            rospy.sleep(self._mission_wait_ms)
        if control is not None:
            self._control_pub.publish(control)
        return control, mission

    def _make_control_and_mission(self, body):
        if 'command' not in body or body['command'] not in ('start', 'stop', 'suspend'):
            return None, None, None

        cmd = body['command']

        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = cmd
        control = Control()
        control.header = h
        if cmd == 'start':
            control.command = 1
            mission = self._make_mission(body)
            return control, mission
        elif cmd == 'stop':
            control.command = 0
            return control, None
        elif cmd == 'suspend':
            control.command = 2
            return control, None
        else:
            rospy.logerr('invalid command {}'.format(body['command']))
            return None, None

    def _make_mission(self, body):
        mission = Mission()
        mission.header.stamp = rospy.Time.now()
        mission.header.frame_id = 'mission'

        details = []
        for wp in body.get('waypoints', []):
            waypint = Detail()
            waypint.command = 1
            waypint.lat = wp['point']['latitude']
            waypint.lng = wp['point']['longitude']
            waypint.param1 = wp['speed'] if wp.get('speed') else 0.0
            waypint.param2 = 0.0
            waypint.param3 = 0.0
            waypint.param4 = 0.0
            details.append(waypint)
            if 'theta' in wp['angle'] and wp['angle']['theta'] is not None:
                theta = Detail()
                theta.command = 3
                theta.lat = 0.0
                theta.lng = 0.0
                theta.param1 = wp['angle']['theta']
                theta.param2 = 0.0
                theta.param3 = 0.0
                theta.param4 = 0.0
                details.append(theta)
            if 'metadata' in wp and 'map' in wp['metadata'] and wp['metadata']['map'] is not None:
                mp = Detail()
                mp.command = 2
                mp.lat = 0.0
                mp.lng = 0.0
                mp.param1 = 0.0
                mp.param2 = wp['metadata']['map']
                mp.param3 = 0.0
                mp.param4 = 0.0
                details.append(mp)
            delay = Detail()
            delay.command = 2
            delay.lat = 0.0
            delay.lng = 0.0
            delay.param1 = wp['metadata']['delay'] if 'metadata' in wp and wp['metadata'].get('delay') else 0.0
            delay.param2 = 0.0
            delay.param3 = 0.0
            delay.param4 = 0.0
            details.append(delay)
        mission.details = details[:-1]
        return mission


def main():
    rospy.init_node('iot_consumer', anonymous=True, disable_signals=True)
    params = wrap_namespace(rospy.get_param('~'))

    control_pub = rospy.Publisher(params.topic.control_cmd, Control, queue_size=1)
    mission_pub = rospy.Publisher(params.topic.mission_cmd, Mission, queue_size=1)
    cmdexe_pub = rospy.Publisher(params.topic.navi_cmdexe, String, queue_size=1)

    naviCommand = NaviCommand(control_pub, mission_pub, cmdexe_pub)
    dispatcher = Dispatcher(naviCommand)
    consumer = Consumer(dispatcher.dispatch_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        consumer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(consumer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
