#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import UInt16


class MoveServoState(EventState):
    """
    Move a servo by publishing a value to a topic.

    -- servo_topic 	   string 	Topic on which to publish the servo position.  
    -- servo_position  UInt16   Position to publish.
    -- wait_time       float    Time to wait after the action.

    <= done 			    Command published.
    """

    def __init__(self, servo_topic, servo_position, wait_time):
        super(MoveServoState, self).__init__(outcomes=['done'])
        # Parameter
        self._servo_topic = servo_topic
        self._servo_position = servo_position
        self._wait_time = rospy.Duration(wait_time)
        # Proxy
        self._servo_pub = ProxyPublisher({self._servo_topic: UInt16})
        self._start_time = None

    def execute(self, userdata):
        if rospy.Time.now() - self._start_time > self._wait_time:
            return 'done'

    def on_enter(self, userdata):
        Logger.loginfo('Publishing %d on topic %s' % (self._servo_position, self._servo_topic))
        self._servo_pub.publish(self._servo_topic, UInt16(self._servo_position))
        self._start_time = rospy.Time.now()