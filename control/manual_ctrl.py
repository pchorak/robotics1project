#! /usr/bin/env python

import time
import curses
from DobotSerialInterface import DobotSerialInterface

dobot_interface = DobotSerialInterface('/dev/tty.usbmodemFD121')

print "Opened connection"
dobot_interface.set_speed()
dobot_interface.set_playback_config()

screen = curses.initscr()

inc = 5.0
angles = [0.0,0.0,0.0]

time.sleep(2)
#dobot_interface.set_initial_angles(angles[1], angles[2])

# print "SENDING FIRST COMMAND"
while dobot_interface.is_connected():
    screen.refresh()
    key = screen.getch()
    if (key==97):
        angles[0] -= inc
    if (key==115):
        angles[1] -= inc
    if (key==100):
        angles[2] -= inc
    if (key==113):
        angles[0] += inc
    if (key==119):
        angles[1] += inc
    if (key==101):
        angles[2] += inc
    if (key==103):
        break
    dobot_interface.send_absolute_angles(angles[0], angles[1], angles[2], 0.0)
    time.sleep(1)

curses.endwin()
