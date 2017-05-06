#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class WaitState(EventState):
    """
    Wait until the given target_time has passed since the behavior has been started.

    -- target_time 	float 	Time which needs to have passed since the behavior started.

    <= continue 			Given time has passed.
    """

    def __init__(self, target_time):
        super(WaitState, self).__init__(outcomes=['continue'])
        self._target_time = rospy.Duration(target_time)
        self._start_time = None

    def execute(self, userdata):
        if rospy.Time.now() - self._start_time > self._target_time:
            return 'continue'

    def on_enter(self, userdata):
        self._start_time = rospy.Time.now()
        time_to_wait = (self._target_time - (rospy.Time.now() - self._start_time)).to_sec()

        if time_to_wait > 0:
            Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)
