import urx
import math3d as m3d
from time import sleep

robot = urx.Robot("192.168.8.4")

pos = robot.get_pos()
print(pos)
robot.close()