#!/usr/bin/env python

import time
import curses
import SerialInterface

interface = SerialInterface.SerialInterface('/dev/tty.usbmodemFD121')

print "Opened connection"
interface.set_speed()
interface.set_playback_config()

screen = curses.initscr()
curses.cbreak()
curses.noecho(1)
screen.keypad(1)

inc = 5.0
angles = [0.0,0.0,0.0]

time.sleep(2)
interface.send_absolute_angles(0.0,0.0,0.0,0.0)
time.sleep(2)
#interface.set_initial_angles(angles[1], angles[2])

# print "SENDING FIRST COMMAND"
while interface.is_connected():
    #screen.refresh()
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
    key = screen.getch()
    # curses.KEY_RIGHT
    # curses.KEY_LEFT
    # curses.KEY_UP
    # curses.KEY_DOWN
    # curses.KEY_EXIT
    interface.send_absolute_angles(angles[0], angles[1], angles[2], 0.0)
    time.sleep(1)

curses.endwin()
