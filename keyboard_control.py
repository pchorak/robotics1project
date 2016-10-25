#!/usr/bin/env python

import time
import curses

import SerialInterface
# import DobotModel

interface = SerialInterface.SerialInterface('/dev/tty.usbmodemFA131')

print "Opened connection"
interface.set_speed()
interface.set_playback_config()

# start screen to read keys
screen = curses.initscr()
curses.cbreak()
curses.noecho()
screen.keypad(1)

# state information
th_inc = 10.0 # deg
p_inc = 10.0 # mm
angles = [0.0,0.0,0.0]
mode = 1; # +/- controls joint 1, 2, or 3

time.sleep(2)
interface.send_absolute_angles(0.0,0.0,0.0,0.0)
time.sleep(2)
#interface.set_initial_angles(angles[1], angles[2])

while interface.is_connected():
    #screen.refresh()
    dth = 0
    dx = 0
    dy = 0
    dz = 0

    c = screen.getch()
    if (c == curses.KEY_EXIT) or (c == 113):
        break
    elif (c == 61): # =
        dth = th_inc
    elif (c == 45): # -
        dth = -th_inc
    elif (c == curses.KEY_RIGHT):
        dy = p_inc
    elif (c == curses.KEY_LEFT):
        dy = -p_inc
    elif (c == curses.KEY_UP):
        dx = p_inc
    elif (c == curses.KEY_DOWN):
        dx = -p_inc
    elif (c == 93): # ]
        dz = p_inc
    elif (c == 91): # [
        dz = -p_inc
    elif (c == 49): # 1
        mode = 1
    elif (c == 50): # 2
        mode = 2
    elif (c == 51): # 3
        mode = 3
    else:
        blah

    if (dth != 0):
        angles[mode-1] += dth
    # elif (dx != 0) or (dy != 0) or (dz != 0):
    #     p = DobotModel.forward_kinematics(angles)
    #     p[0] += dx
    #     p[1] += dy
    #     p[2] += dz
    #     angles = DobotModel.inverse_kinematics(p)

    interface.send_absolute_angles(angles[0], angles[1], angles[2], 0.0)
    time.sleep(1)
    angles = interface.current_status.get_angles()

curses.endwin()
