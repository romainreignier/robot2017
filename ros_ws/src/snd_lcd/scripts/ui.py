#!/usr/bin/env python

# Pygame imports
import os
import sys
import pygame
from math import *
from pygame.locals import *

# ROS imports
import rospy
import rospkg
from snd_msgs.msg import Status, Color
from std_msgs.msg import ColorRGBA

# Define some common colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)


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
        self.screen.blit(self.bg, (0, 0))

        # Get system font
        self.font = pygame.font.Font(None, 30)

        # Init ros node
        rospy.init_node('snd_ui')
        rospy.loginfo('User interface launched.')

        # Subscribe to some topics
        self.status_sub = rospy.Subscriber('/status', Status, self.status_cb)
        self.status = None
        self.color_sub = rospy.Subscriber('/color_sensor', ColorRGBA, self.color_cb)
        self.color_msg = None

    def color_cb(self, msg):
        self.color_msg = msg

    def status_cb(self, msg):
        self.status = msg

    def RGB_compare(self, R_r, G_r, B_r, color):
        if color == Color.BLUE:
            R_b = 0
            G_b = 0
            B_b = 255
            d = sqrt((R_b - R_r) * (R_b - R_r) + (G_b - G_r) * (G_b - G_r) + (B_b - B_r) * (B_b - B_r))
            p = d / sqrt((255) ^ 2 + (255) ^ 2 + (255) ^ 2)
            print('p = {}'.format(p))
            if p < 14:  # chiffre totalement arbitraire
                self.screen.blit(self.font.render('Couleur Bleu !', True, (BLUE)), (20, 150))
            if p >= 14:
                self.screen.blit(self.font.render('MAUVAISE COULEUR ! (jaune)', True, (RED)), (20, 150))
        if color == Color.YELLOW:
            R_j = 255
            G_j = 255
            B_j = 0
            d = sqrt((R_j - R_r) * (R_j - R_r) + (G_j - G_r) * (G_j - G_r) + (B_j - B_r) * (B_j - B_r))
            p = d / sqrt((255) ^ 2 + (255) ^ 2 + (255) ^ 2)
            print('p = {}'.format(p))
            if p < 14:
                self.screen.blit(self.font.render('Couleur Jaune !', True, (YELLOW)), (20, 150))
            if p >= 14:
                self.screen.blit(self.font.render('MAUVAISE COULEUR ! (bleu)', True, (RED)), (20, 150))

    def run(self):
        # self.screen.fill(WHITE)
        self.screen.blit(self.bg, (0, 0))
        # pygame.draw.line(self.screen, BLUE, (20, 230), (300, 230), 3)
        if self.status is not None:
            if self.status.starter:
                starter_text = self.font.render('Tirette mise', True, (RED))
                self.screen.blit(starter_text, (20, 20))
                if self.color_msg is not None:
                    ui.RGB_compare(self.color_msg.r,
                                   self.color_msg.g,
                                   self.color_msg.b,
                                   self.status.color_switch.color)
            else:
                starter_text = self.font.render('Tirette non mise', True, (BLUE))
                self.screen.blit(starter_text, (20, 20))
            if self.status.eStop:
                estop_text = self.font.render('Arret d\'urgence actif !', True, (RED))
                self.screen.blit(estop_text, (20, 60))
            else:
                estop_text = self.font.render('Arret d\'urgence NON actif', True, (BLUE))
                self.screen.blit(estop_text, (20, 60))


if __name__ == '__main__':
    ui = Ui()
    # Application loop
    r = rospy.Rate(2)  # 2 Hz
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
