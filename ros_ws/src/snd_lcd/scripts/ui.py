#!/usr/bin/env python

# Pygame imports
import os
import sys
import pygame
from pygame.locals import *

# ROS imports
import rospy
import rospkg
from snd_msgs.msg import Status

# Define some common colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


class Ui:
    def __init__(self):
        # Check if we are on X or not to set the framebuffer
        disp_no = os.getenv("DISPLAY")
        if not disp_no:
            # There is no DISPLAY env variable so we are not on a X server
            os.environ["SDL_FBDEV"] = "/dev/fb8"
            print('On framebuffer')
        else:
            print('On X server')

        # Initialize pygame
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((320, 240), 0, 32)
        pygame.display.set_caption('Drawing')
        self.screen.fill(WHITE)
        pygame.display.update()

        # Load the background image
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('snd_lcd')
        self.bg = pygame.image.load(pkg_path + '/scripts/snd_logo.png').convert()
        self.screen.blit(self.bg, (0,0))

        # Get system font
        self.font = pygame.font.Font(None, 30)

        # Init ros node
        rospy.init_node('snd_ui')
        rospy.loginfo('User interface launched.')

        # Subscribe to some topics
        self.status_sub = rospy.Subscriber('/status', Status, self.status_cb)
        self.status = None

    def status_cb(self, msg):
        self.status = msg

    def run(self):
        # self.screen.fill(WHITE)
        self.screen.blit(self.bg, (0, 0))
        pygame.draw.line(self.screen, BLUE, (20, 230), (300, 230), 3)
        if self.status is not None:
            if self.status.starter:
                starter_text = self.font.render('Tirette mise', True, (RED))
                self.screen.blit(starter_text, (20, 20))
            else:
                starter_text = self.font.render('Tirette non mise', True, (BLUE))
                self.screen.blit(starter_text, (20, 20))


if __name__ == '__main__':
    ui = Ui()
    # Application loop
    r = rospy.Rate(2) # 2 Hz
    while not rospy.is_shutdown():
        # Quit if Quit event received
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print('Quit')
                rospy.signal_shutdown('Pygame exit')
        # Run the app
        ui.run()
        # Update the screen
        pygame.display.update()
        # Sleep
        r.sleep()
    pygame.quit()
