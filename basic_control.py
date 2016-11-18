#!/usr/bin/env python

# Purpose: Use the keyboard to directly increment and decrement the Dobot's joint angles
# in order to find the angle limits.

import time
import curses
import os.path

import SerialInterface

if __name__ == '__main__':
    ports = ['/dev/tty.usbmodemFD121','/dev/tty.usbmodemFA131','/dev/ttyACM0']
    interface = None
    for port in ports:
        if os.path.exists(port):
            interface = SerialInterface.SerialInterface(port)
    if interface is None:
        print "Serial device not found in known list"
    else:
        screen = curses.initscr()
        curses.cbreak()
        curses.noecho()
        #screen.nodelay(1)
        screen.keypad(1)

        speed = 30

        time.sleep(1)
        interface.send_absolute_angles(0,20,20,0)
        time.sleep(1)

        while interface.is_connected():
            screen.clear()
            screen.move(0,0)
            screen.addstr("(%.2f,%.2f,%.2f)" % tuple(interface.current_status.get_angles()))
            screen.refresh()

            c = screen.getch()
            if (c == 113): # q
                break
            elif (c == 61): # =
                speed = min(100,speed+5)
            elif (c == 45): # -
                speed = max(0,speed-5)
            elif (c == curses.KEY_LEFT):
                interface.send_jog_command(False,1,speed)
            elif (c == curses.KEY_UP):
                interface.send_jog_command(False,3,speed)
            elif (c == 91): # [
                interface.send_jog_command(False,5,speed)
            elif (c == curses.KEY_RIGHT):
                interface.send_jog_command(False,2,speed)
            elif (c == curses.KEY_DOWN):
                interface.send_jog_command(False,4,speed)
            elif (c == 93): # ]
                interface.send_jog_command(False,6,speed)
            else:
                interface.send_jog_command(False,0,0)
            time.sleep(0.1)

        curses.endwin()
