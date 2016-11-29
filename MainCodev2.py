import AR_Camera
import time
import sys
import threading
import numpy as np
import DobotModel
import SerialInterface

#=====DEFINED CONSTANTS=====
CAMERA_ID = 1
VIDEO_MODE = True
DUCKY = [16273625, 25]
DUCKYBOT = [56356251, 25]
OBSTACLE = [425234, 51]
REGISTERED_TAGS = [DUCKY, DUCKYBOT, OBSTACLE]
CAMERA_OFFSET = [[-9.059], [-24.953], [30.019]]
DUCKY_POS = np.reshape(np.array([149.66,-228.5,0]) ,(3, 1))
# z = 5.04

# Display poses of all objects
# [ [Vector tag to camera] ,  [Rotation of tag] ]
# If not present, [None, None] will be reported
def get_data(camera):
    print "----------------------------"
    print "Ducky Pose:", camera.Ducky_Pose
    print "Duckybot Pose:", camera.Duckybot_Pose
    print "Obstacle Pose:", camera.Obstacle_Pose
    print "----------------------------"



# Return the Dobot joint angles needed to go to desired XYZ
def get_angles(coordinates):
    return DobotModel.inverse_kinematics(coordinates)



# Move Dobot to a desired XYZ position
def move_xyz(interface, target, pump_on = False):
    angles = get_angles(target)
    if any(np.isnan(angles)):
        print "Error: No solution for coordinates: ", target
    else:
        interface.send_absolute_angles(angles[0],angles[1],angles[2],0.0, interface.MOVE_MODE_JOINTS, pump_on)


# get xyz of tag_index
def get_xyz(interface, tag_index):
    angles = interface.current_status.angles[0:3]

    # Get current XYZ
    P0t = DobotModel.forward_kinematics(angles)
    # Renew data
    data = camera.get_all_poses()[tag_index]

    # Getting Desired XYZ of end effector
    Pct = np.array(CAMERA_OFFSET)
    R0t = DobotModel.R0T(angles)
    Rtc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    R0c = np.matmul(R0t, Rtc)
    Pta = np.matmul(R0c, data[0]) - np.matmul(R0c, Pct)

    target = np.reshape(Pta, (3, 1)) + np.reshape(P0t, (3, 1))

    return target

# Touch the end effector to a detected object
def touch(interface, tag_index, mode, pump_on = False):
    angles = interface.current_status.angles[0:3]

    # Get current XYZ
    P0t = DobotModel.forward_kinematics(angles)


    # Renew data
    data = camera.get_all_poses()[tag_index]

    # If arial mode and z is low, move up a bit
    # NO LONGER NEEDED
    if mode == 2 and P0t[2] < 0:
        P0t = P0t + [0,0,50]
        move_xyz(interface, P0t, pump_on)

    # Getting Desired XYZ of end effector
    Pct = np.array(CAMERA_OFFSET)
    R0t = DobotModel.R0T(angles)
    Rtc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    R0c = np.matmul(R0t, Rtc)
    Pta = np.matmul(R0c, data[0]) - np.matmul(R0c, Pct)

    target = np.reshape(Pta, (3, 1)) + np.reshape(P0t, (3, 1))

    # DIRECT MODE
    if mode == 1:
        # Move directly to target
        move_xyz(interface, target, pump_on)

    # ARIAL MODE
    elif mode == 2:
        # Move directly above target
        move_xyz(interface, target + np.reshape(np.array([0,0,50]) ,(3, 1)))
        # Move to target
        move_xyz(interface, target, pump_on)



# Track an AR Tag. DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
def track(interface, camera, tag_index):
    # listen for user input
    req_exit = []
    listener = threading.Thread(target=input_thread, args=(req_exit,))
    listener.start()

    time.sleep(1)

    while not req_exit:
        # Only enter search if capture_data failed to find tag for 10 consecutive frames
        searching = True
        for x in range(0,30):
            time.sleep(0.1)
            data = camera.get_all_poses()[tag_index]
            if data != [None, None]:
                searching = False
                break;

        if searching:
            # Search until you find the desired tag
            data = search(interface, camera, tag_index)

        # User requested to exit during search process
        if data == None:
            return

        # Follow tag while it is in view
        while data != [None, None] and not req_exit:
                # From kinematics
                angles = interface.current_status.angles[0:3]
                P0t = np.reshape(DobotModel.forward_kinematics(angles), (3,1))
                R0t = DobotModel.R0T(angles)

                # From calibration
                Rtc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
                R0c = np.matmul(R0t,Rtc)

                # From camera
                Pca = np.reshape(data[0], (3,1))
                # Hover above it (Z offest 5 * marker size)
                Pca_des = np.array([[0], [0], [5 * REGISTERED_TAGS[tag_index][1]]])
                correction = -1*np.matmul(R0c, Pca_des - Pca)

                # If the change in desired XYZ is notable, move to track it
                if np.linalg.norm(correction) > 2.5:
                    angles = move_xyz(interface, P0t + correction)
                    time.sleep(0.5)

                # Get data
                data = camera.get_all_poses()[tag_index]


# Search for AR Tag. DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
# If tag_index is left undefined , it will search until any registered tag is found
def search(interface, camera, tag_index = -1):

    base_angle = interface.current_status.get_base_angle()
    direction = 1

    # listen for user input
    req_exit = []
    listener = threading.Thread(target=input_thread, args=(req_exit,))
    listener.start()

    while not req_exit:
        if direction == 1:
            while base_angle < 100 :
                if req_exit:
                    return None

                # Get data
                data = camera.get_all_poses()

                print data

                # Search all tags
                if tag_index == -1:
                    for x in range(0,3):
                        if data[x] != [None, None]:
                            return data[x]
                else:
                    # Search for a specified tag
                    if data[tag_index] != [None, None]:
                        return data[tag_index]

                base_angle = base_angle + 10

                interface.send_absolute_angles(base_angle, 10, 10, 0)

                time.sleep(1)
        else:
            while base_angle > -100 :
                if req_exit:
                    return None

                # Get data
                data = camera.get_all_poses()

                print data

                # Search all tags
                if tag_index == -1:
                    for x in range(0,3):
                        if data[x] != [None, None]:
                            return data[x]
                else:
                    # Search for a specified tag
                    if data[tag_index] != [None, None]:
                        return data[tag_index]

                base_angle = base_angle - 10

                interface.send_absolute_angles(base_angle, 10, 10, 0)

                time.sleep(1)

        # change direction
        direction = direction * -1

    # user requested to exit
    interface.send_absolute_angles(10, 10, 10, 0)
    return None


# USED FOR GETTING INPUT TO BREAK LOOPS
def input_thread(usr_list):
    raw_input()
    usr_list.append(None)





if __name__ == '__main__':

    # INTITIALIZING SERIAL INTERFACE
    interface = SerialInterface.SerialInterface('/dev/ttyACM0') # Linux

    time.sleep(1)
    interface.send_absolute_angles(0,10,10,0)
    time.sleep(1)

    # SET INITIAL XYZ:
    initial_xyz = DobotModel.forward_kinematics([0,10,10,0])


    # INITIALIZING CAMERA
    sys.stdout.write("Initiliazing the Camera..." )

    camera = AR_Camera.Camera(CAMERA_ID, VIDEO_MODE, DUCKY, DUCKYBOT, OBSTACLE)
    camera.start()

    print "OK!\n"

    # DISPLAY CAMERA PROPERTIES
    print camera

    object_selection = '''OBJECTS:
    1. ducky
    2. duckybot
    3. obstacle
    '''

    options = '''COMMANDS:
    reset
    top view
    get data
    touch
    track
    grab ducky
    place ducky
    arm calibration
    quit
    '''
    while True:
        print options
        command = raw_input("Enter Command => ").lower()

        if command == "reset":
            interface.send_absolute_angles(0,10,10,0)

        elif command == "arm calibration":
            angles = interface.current_status.angles[0:3]
            # Get current XYZ
            P0t = DobotModel.forward_kinematics(angles)
            goal = [P0t[0],P0t[1], -20]
            move_xyz(interface, goal)

            time.sleep(5)

            goal2 = np.reshape(goal ,(3, 1)) + np.reshape(np.array([0,150,0]) ,(3, 1))

            move_xyz(interface, goal2)



        elif command == "top view":
            # Get current XYZ
            P0t = DobotModel.forward_kinematics(interface.current_status.get_angles())
            P0t[2] = 100
            move_xyz(interface, P0t)
            # Allow the camera to refocus
            time.sleep(2)

        elif command == "get data":
            get_data(camera)

        elif command == "grab ducky":

            target = DUCKY_POS + np.array([[0], [0], [30]])
            move_xyz(interface, target, True)
            time.sleep(1)

            move_xyz(interface, DUCKY_POS, True)

            time.sleep(2)
            interface.send_absolute_angles(0,10,10,0, interface.MOVE_MODE_JOINTS, True)

        elif command == "place ducky":
            # Which object to place ducky on?
            print object_selection
            selection = int(input("Which object to place ducky on: "))
            # Get data
            data = camera.get_all_poses()[selection - 1]
            if data != [None,None]:
                goal_xyz = get_xyz(interface, selection - 1) + np.array([[0], [0], [45]])

                ducky_xyz = DUCKY_POS + np.array([[0], [0], [30]])
                move_xyz(interface, ducky_xyz, True)
                time.sleep(1)

                move_xyz(interface, DUCKY_POS, True)

                time.sleep(2)

                move_xyz(interface, ducky_xyz, True)
                
                time.sleep(1)

                move_xyz(interface, goal_xyz, True)

                time.sleep(2)

                move_xyz(interface, goal_xyz, False)
            else:
                print "Object is not available."

        elif command == "touch":
            # Which object to touch?
            print object_selection
            selection = int(input("Which object to touch: "))
            # Get data
            data = camera.get_all_poses()[selection - 1]
            # Touch the object if it is available
            if data != [None,None]:
                touch(interface, selection - 1, 1)
            else:
                print "Object is not available."

        elif command == "track":
            # Which object to track?
            print object_selection
            selection = int(input("Which object to track: "))
            # Begin Tracking
            track(interface, camera, selection - 1)

        elif command == "quit":
            interface.send_absolute_angles(0,10,10,0)
            sys.stdout.write("Releasing camera...")
            camera.release()
            print "OK!"
            break;
        elif command == "test":
            interface.send_absolute_angles(-68, 10, 10, 0)
            time.sleep(2)
            interface.send_absolute_angles(0, 10, 10, 0)
            time.sleep(2)
            interface.send_absolute_angles(68, 10, 10, 0)
