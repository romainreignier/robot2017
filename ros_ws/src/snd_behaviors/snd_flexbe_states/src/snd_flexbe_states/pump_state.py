#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import Bool


class PumpState(EventState):
    """
    Turn On/Off the vacuum pump by publishing a boolean to a topic.

    -- topic 	   string 	Topic on which to publish the pump state.  
    -- state       boolean  State to publish.
    -- wait_time   float    Time to wait after the action.

    <= done                 Command published.
    """

    def __init__(self, topic, state, wait_time):
        super(PumpState, self).__init__(outcomes=['done'])
        # Parameters
        self._topic = topic
        self._state = state
        self._wait_time = rospy.Duration(wait_time)
        # Proxy
        self._pub = ProxyPublisher({self._topic: Bool})
        self._start_time = None

    def execute(self, userdata):
        if rospy.Time.now() - self._start_time > self._wait_time:
            return 'done'

    def on_enter(self, userdata):
        Logger.loginfo('Publishing %s on topic %s' % (self._state, self._topic))
        self._pub.publish(self._topic, Bool(self._state))
        self._start_time = rospy.Time.now()