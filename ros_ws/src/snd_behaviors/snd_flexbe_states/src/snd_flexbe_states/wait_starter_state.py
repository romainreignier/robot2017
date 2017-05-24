#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from snd_msgs.msg import Status


class WaitStarterState(EventState):
    """
    Wait that the starter is removed from the robot.

    -- status_topic  string  Name of the status topic.

    <= start                 The starter has been removed.
    """

    def __init__(self, status_topic):
        super(WaitStarterState, self).__init__(outcomes=['start'])
        self._topic = status_topic
        self._sub = ProxySubscriberCached({self._topic: Status})

    def execute(self, userdata):
        if self._sub.has_msg(self._topic):
            msg = self._sub.get_last_msg(self._topic)
            if not msg.starter:
                return 'start'
