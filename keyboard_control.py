#!/usr/bin/env python

import time
import curses
import numpy as np
import os.path
import argparse

import DobotModel
import Controller
import AR_Camera

DUCKY = [16287382, 51]



def keyboard_control(port):
        controller = Controller.Controller(port)
        
        cam = AR_Camera.Camera(0,DUCKY)
        cam.initialize()

        # start screen to read keys
        screen = curses.initscr()
        curses.cbreak()
        curses.noecho()
        screen.nodelay(1) # DIFFERENT #
        screen.keypad(1)

        angle_list = []
        pca_list = []

        vid = False # video state

        while controller.is_connected():
            c = screen.getch()
            if (c == 113): # q
                controller.stop()
                time.sleep(1)
                break
            elif (c == 99): # c
                time.sleep(1)
                angle_list.append(controller.get_angles())
                pose,_,_,_ = cam.capture_data()
                pca_list.append(pose[0])
            elif (c == 109): # m
                controller.switch_modes()
            elif (c == 118): # v
                if not vid:
                    cam.activate_video()
                    vid = True
                else:
                    cam.deactivate_video()
                    vid = False
            elif (c == 61): # =
                controller.change_effort(5)
            elif (c == 45): # -
                controller.change_effort(-5)
            elif (c == curses.KEY_LEFT):
                controller.move(1)
            elif (c == curses.KEY_UP):
                controller.move(3)
            elif (c == 91): # [
                controller.move(5)
            elif (c == curses.KEY_RIGHT):
                controller.move(2)
            elif (c == curses.KEY_DOWN):
                controller.move(4)
            elif (c == 93): # ]
                controller.move(6)
            elif (c > 0):
                controller.stop()

            time.sleep(0.1)

            screen.clear()
            screen.move(0,0)
            screen.addstr("(%.2f,%.2f,%.2f)" % tuple(controller.get_angles()))
            screen.move(1,0)
            screen.addstr("[%.2f,%.2f,%.2f]" % tuple(controller.get_position()))
            if (controller.mode == 0):
                screen.move(2,0)
                screen.addstr("(%.2f,%.2f,%.2f)" % tuple(controller.angles))
                screen.move(3,0)
                screen.addstr("[%.2f,%.2f,%.2f]" % tuple(DobotModel.forward_kinematics(controller.angles)))
            screen.refresh()

        curses.endwin()
        cam.release()
        return (angle_list,pca_list)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', type=None, default='/dev/ttyACM0')
    args = parser.parse_args()
    if not os.path.exists(args.port):
        print "Serial device '%s' not found" % args.port
    else:
        keyboard_control(args.port)
