"""
Joystick control
"""

import time
import socket
import pygame

import math3d as m3d
from math import pi

import urx

class Cmd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0
        self.axis3 = 0
        self.btn0 = 0
        self.btn1 = 0
        self.btn2 = 0
        self.btn3 = 0
        self.btn4 = 0
        self.btn5 = 0
        self.btn6 = 0
        self.btn7 = 0
        self.btn8 = 0
        self.btn9 = 0
        self.btn10 = 0
        self.btn11 = 0
        self.hat0 = [0,0]


class Service(object):
    def __init__(self, linear_velocity=0.1, rotational_velocity=0.1, acceleration=0.1):
        global robot
        self.joystick = None
        self.robot = robot
        #max velocity and acceleration to be send to robot
        self.linear_velocity = linear_velocity
        self.rotational_velocity = rotational_velocity
        self.acceleration = acceleration
        #one button send the robot to a preprogram position defined by this variable in join space
        #self.init_pose = [-2.0782002408411593, -1.6628931459654561, 2.067930303382134, -1.9172217394630149, 1.5489023943220621, 0.6783171005488982]

        self.cmd = Cmd()


    def init_joystick(self):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(1)
        self.joystick.init()
        print('Initialized Joystick : %s' % self.joystick.get_name())

    def loop(self, initiated_by = None):
        print("Starting diagnost loop")
        air = False
        routing = False
        while True:
            self.cmd.reset()
            pygame.event.pump()#Seems we need polling in pygame...

            #get joystick state
            for i in range(0, self.joystick.get_numaxes()):
                val = self.joystick.get_axis(i)
                if i in (2, 5) and val != 0:
                    val += 1
                if abs(val) < 0.2:
                    val = 0
                tmp = "self.cmd.axis" + str(i) + " = " + str(val)
                if val != 0:
                    #print(tmp)
                    exec(tmp)

            #get button state
            for i in range(0, self.joystick.get_numbuttons()):
                if self.joystick.get_button(i) != 0:
                    tmp = "self.cmd.btn" + str(i) + " = 1"
                    #print(tmp)
                    exec(tmp)

            if self.cmd.btn10 and not routing:
                initial_pos = robot.get


            #initalize speed array to 0
            speeds = [0, 0, 0, 0, 0, 0]

            #get linear speed from joystick
            speeds[1] = 1 * self.joystick.get_hat(0)[1] * self.linear_velocity
            speeds[0] = 1 * self.joystick.get_hat(0)[0] * self.linear_velocity
            if self.cmd.btn2 and not self.cmd.btn3:
                speeds[2] = 1 * -self.linear_velocity
            if self.cmd.btn3 and not self.cmd.btn2:
                speeds[2] = 1 * self.linear_velocity

            #get rotational speed from joystick
            speeds[3] = -1 * self.cmd.axis1 * self.rotational_velocity
            speeds[4] = -1 * self.cmd.axis0 * self.rotational_velocity
            speeds[5] = self.cmd.axis3 * self.rotational_velocity


            #for some reasons everything is inversed
            speeds = [-i for i in speeds]
            #Now sending to robot. tol by default and base csys if btn2 is on
            #if speeds != [0 for _ in speeds]:
                #print("Sending ", speeds)

            if self.cmd.btn0:
                self.robot.speedl_tool(speeds, 0.1, 2)
            else:
                self.robot.speedl(speeds, 0.1, 2)

            #s.send((cmd).encode())
            #data = s.recv(1024)

            if initiated_by:
                if initiated_by.auto == False:
                    break

    def close(self):
        if self.joystick:
            self.joystick.quit()


def main(initiated_by = None):
    global robot
    robot = urx.Robot("192.168.8.3")
    r = robot


    service = Service(linear_velocity=0.1, rotational_velocity=0.19, acceleration=0.1)
    service.init_joystick()
    try:
        service.loop(initiated_by)
    finally:
        print('Джойстик диагноста отключён')
        service.close()

if __name__ == "__main__":
    robot = urx.Robot("192.168.8.3")
    r = robot


    service = Service(linear_velocity=0.1, rotational_velocity=0.19, acceleration=0.1)
    service.init_joystick()
    try:
        service.loop()
    finally:
        service.close()
