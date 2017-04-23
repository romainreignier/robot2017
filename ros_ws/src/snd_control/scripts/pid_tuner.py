#!/usr/bin/env python2

import rospy

from dynamic_reconfigure.server import Server
from snd_control.cfg import PidConfig
from snd_msgs.msg import Encoders, Motors, Pid
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

JOY_RATIO = 7345


class PidTuner:
    def __init__(self):
        # Publishers
        self.motors_pub = rospy.Publisher('/motors_speed', Motors, queue_size=1)
        self.left_motor_pid_pub = rospy.Publisher('/left_motor_pid', Pid, queue_size=1)
        self.right_motor_pid_pub = rospy.Publisher('/right_motor_pid', Pid, queue_size=1)
        self.encoders_speed_pub = rospy.Publisher('/encoders_speed', Motors, queue_size=1)
        # Subscribers
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.encoders_sub = rospy.Subscriber('/encoders', Encoders, self.encoders_cb)
        # Dynamic Reconfigure
        self.srv = Server(PidConfig, self.pid_cb)
        # Timer to publish the command regularly
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        # Variables
        self.left_command = 0
        self.right_command = 0
        self.left_speed = 0
        self.right_speed = 0
        self.command_inc = 800
        self.first_run = True
        self.previous_time = rospy.get_rostime()
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0

    def joy_cb(self, msg):
        # left motor
        self.left_command = float(msg.axes[3] * JOY_RATIO)
        # right motor
        self.right_command = float(msg.axes[4] * JOY_RATIO)
        rospy.loginfo(
            'Sending motor command left: {} right: {}'.format(self.left_command, self.right_command))

    def timer_cb(self, event):
        motors_command = Motors()
        motors_command.header.stamp = rospy.get_rostime()
        if self.left_speed <  self.left_command - self.command_inc:
            self.left_speed += self.command_inc
        elif self.left_speed > self.left_command + self.command_inc:
            self.left_speed -= self.command_inc
        else:
            self.left_speed = self.left_command

        if self.right_speed < self.right_command - self.command_inc:
            self.right_speed += self.command_inc
        elif self.right_speed > self.right_command + self.command_inc:
            self.right_speed -= self.command_inc
        else:
            self.right_speed = self.right_command

        motors_command.left = self.left_speed
        motors_command.right = self.right_speed
        self.motors_pub.publish(motors_command)

    def pid_cb(self, config, level):
        if level & 1:
            rospy.loginfo(
                'Pid reconfigure request left motor: P: {left_speed_p} I: {left_speed_i} D: {left_speed_d}'.format(
                    **config))
            self.left_motor_pid_pub.publish(Pid(config.left_speed_p, config.left_speed_i, config.left_speed_d))
        if level & 2:
            rospy.loginfo(
                'Pid reconfigure request right motor: P: {right_speed_p} I: {right_speed_i} D: {right_speed_d}'.format(
                    **config))
            self.right_motor_pid_pub.publish(Pid(config.right_speed_p, config.right_speed_i, config.right_speed_d))
        return config

    def encoders_cb(self, msg):
        time = msg.header.stamp
        left_ticks = msg.left
        right_ticks = msg.right
        left_speed = float((left_ticks - self.previous_left_ticks) / (time - self.previous_time).to_sec())
        right_speed = float((right_ticks - self.previous_right_ticks) / (time - self.previous_time).to_sec())
        self.previous_left_ticks = left_ticks
        self.previous_right_ticks = right_ticks
        self.previous_time = time
        if self.first_run is True:
            self.first_run = False
        else:
            self.encoders_speed_pub.publish(Motors(Header(0, time, ' '), left_speed, right_speed))


if __name__ == '__main__':
    # Node Initialization
    rospy.init_node('pid_tuner')
    rospy.loginfo('pid_tuner node started.')
    PidTuner()
    # Keep the node running
    rospy.spin()
