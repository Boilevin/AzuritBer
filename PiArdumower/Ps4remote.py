#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#
# This file presents an interface for interacting with the Playstation 4 Controller
# in Python. Simply plug your PS4 controller into your computer using USB and run this
# script!
#
# NOTE: I assume in this script that the only joystick plugged in is the PS4 controller.
#       if this is not the case, you will need to change the class accordingly.
#
# Copyright © 2015 Clay L. McLeod <clay.l.mcleod@gmail.com>
#
# Distributed under terms of the MIT license.

import os
import pygame

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None
    leftClick=False
    rightClick=False
    upClick=False
    downClick=False
    crossClick=False
    roundClick=False
    squareClick=False
    triangleClick=False
    

    def init(self):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        """Listen for events to happen"""
        self.leftClick=False
        self.rightClick=False
        self.upClick=False
        self.downClick=False
        self.crossClick=False
        self.roundClick=False
        self.squareClick=False
        self.triangleClick=False
    

        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value,2)
                    
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                    if self.button_data[0] :
                        self.crossClick=True
                    if self.button_data[1] :
                        self.roundClick=True
                    if self.button_data[3] :
                        self.squareClick=True
                    if self.button_data[2] :
                        self.triangleClick=True
                    #print("JOYBUTTONdown")    
                    
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                    #print("JOYBUTTONUP")
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value
                    if event.hat == 0:
                        if event.value == (1, 0):
                            self.rightClick=True
                        if event.value == (-1, 0):
                            self.leftClick=True
                        if event.value == (0, 1):
                            self.upClick=True
                        if event.value == (0,-1):
                            self.downClick=True
                            

                #os.system('clear')
                #pprint.pprint(self.button_data)
                #pprint.pprint(self.axis_data)
                #pprint.pprint(self.hat_data)


if __name__ == "__main__":
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()
