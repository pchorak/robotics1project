#! /usr/bin/env python


import time

from DobotSerialInterface import DobotSerialInterface

dobot_interface = DobotSerialInterface()

print "Opened connection"
dobot_interface.set_speed()
dobot_interface.set_playback_config()


sleep_duration = 2


first_round = False
time.sleep(2)
# print "SENDING FIRST COMMAND"
while True:
    # continue
    #set_initial_angles(45, 45)

    if first_round:
        first_round = False
        #  88.62541198730469, -0.9358671307563782
        # dobot_interface.apply_arm_angle_offsets(1.37, 0.936)
        # self.apply_arm_angle_offsets(0, 0, dobot_interface.current_status)
        time.sleep(2)
    else:
        # continue
        # dobot_interface.send_absolute_position(250, 0, 0)  # MOVL
        # time.sleep(sleep_duration)
        # dobot_interface.send_absolute_position(250, 0, 120)  # MOVL
        # time.sleep(sleep_duration)
        print "send angles"

        dobot_interface.send_absolute_angles(-30, 0.0, 0.0, 30)
        # dobot_interface.send_absolute_angles(0, 20.0, 00.0)
        # dobot_interface.send_absolute_angles(0, 10.0, 00.0)
        time.sleep(2)
        dobot_interface.send_absolute_angles(30, 30.0, 00.0, -30)
        time.sleep(2)
