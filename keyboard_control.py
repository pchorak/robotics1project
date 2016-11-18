#!/usr/bin/env python

import time
import curses
import numpy as np
import os.path
import argparse

import SerialInterface
import DobotModel

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', type=None, default='/dev/ttyACM0')
    args = parser.parse_args()
    if not os.path.exists(args.port):
        print "Serial device '%s' not found" % args.port
    else:
        interface = SerialInterface.SerialInterface(args.port)

        # start screen to read keys
        screen = curses.initscr()
        curses.cbreak()
        curses.noecho()
        screen.nodelay(1)
        screen.keypad(1)

        # state information
        th_inc = 2.0 # deg
        p_inc = 5.0 # mm
        angles = [0.0,20,20]
        mode = 1; # +/- controls joint 1, 2, or 3

        time.sleep(1)
        interface.send_absolute_angles(angles[0], angles[1], angles[2], 0.0)
        time.sleep(1)
        #interface.set_initial_angles(angles[1], angles[2])

        while interface.is_connected():
            dth = 0
            dx = 0
            dy = 0
            dz = 0

            c = screen.getch()
            if (c == 113): # q
                break
            elif (c == 61): # =
                dth = th_inc
            elif (c == 45): # -
                dth = -th_inc
            elif (c == curses.KEY_LEFT):
                dy = p_inc
            elif (c == curses.KEY_RIGHT):
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

            if (dth != 0):
                angles[mode-1] += dth
                interface.send_absolute_angles(angles[0], angles[1], angles[2], 0.0)
            elif (dx != 0) or (dy != 0) or (dz != 0):
                p = DobotModel.forward_kinematics(angles)
                p[0] += dx
                p[1] += dy
                p[2] += dz
                tmp = list(DobotModel.inverse_kinematics(p))
                if not any(np.isnan(tmp)):
                    angles = tmp
                    interface.send_absolute_angles(angles[0], angles[1], angles[2], 0.0)

            time.sleep(0.1)

            screen.clear()
            screen.move(0,0)
            screen.addstr("(%.2f,%.2f,%.2f)" % tuple(angles))
            screen.move(1,0)
            screen.addstr("(%.2f,%.2f,%.2f)" % tuple(interface.current_status.get_angles()))
            screen.move(2,0)
            screen.addstr("[%.2f,%.2f,%.2f]" % tuple(DobotModel.forward_kinematics(angles)))
            screen.move(3,0)
            screen.addstr("[%.2f,%.2f,%.2f]" % tuple(interface.current_status.position[0:3]))
            screen.refresh()

        curses.endwin()
