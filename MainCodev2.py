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
DUCKY = [16273625, 25] # normally 16273625
DUCKYBOT = [12345678, 25]
OBSTACLE = [87654321, 25]
REGISTERED_TAGS = [DUCKY, DUCKYBOT, OBSTACLE]
CAMERA_OFFSET = [[-9.059], [-24.953], [30.019]]
DUCKY_POS = np.reshape(np.array([149.66,-228.5,0]) ,(3, 1))
MAXHEIGHT = 115
CLEANUP_POS = np.reshape(np.array([149.66,-228.5,0]) ,(3, 1)) # UPDATE THIS TO BEHIND DUCKY

TEST_TAG_1 = [11111111, 25]
TEST_TAG_2 = [12345678, 25]
TEST_TAG_3 = [87654321, 25]
TEST_TAG_4 = [56473627, 25]


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
def move_xyz(interface, target, pump_on = False, joint_4_angle = 0):
    angles = get_angles(target)
    if any(np.isnan(angles)):
        print "Error: No solution for coordinates: ", target
    else:
        interface.send_absolute_angles(angles[0],angles[1],angles[2], joint_4_angle, interface.MOVE_MODE_JOINTS, pump_on)

# Get required XYZ to move end effector to AR tag
def get_xyz(interface, xyz_from_camera):
    angles = interface.current_status.angles[0:3]

    # Get current XYZ
    P0t = DobotModel.forward_kinematics(angles)

    # Getting Desired XYZ of end effector
    Pct = np.array(CAMERA_OFFSET)
    R0t = DobotModel.R0T(angles)
    Rtc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    R0c = np.matmul(R0t, Rtc)

    Pta = np.matmul(R0c, xyz_from_camera) - np.matmul(R0c, Pct)

    target = np.reshape(Pta, (3, 1)) + np.reshape(P0t, (3, 1))

    return target

# Touch the end effector to a detected object
def touch(interface, tag_index, mode, pump_on = False):

    # Get the object XYZ position of the tag relative to the camera
    object_xyz = camera.get_all_poses()[tag_index][0]

    # Get the target XYZ position
    target = get_xyz(interface, object_xyz)

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

                #print data


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

                # print data

                # Search all tags
                if tag_index == -1:
                    for x in range(0,3):
                        if data[x] != [None, None]:
                            return data[x]
                else:
                    # Search for a specified tag
                    if data[tag_index] != [None, None]:
                        return data[tag_index]

                base_angle = base_angle + 5

                interface.send_absolute_angles(base_angle, 10, 10, 0)

                time.sleep(1)
        else:
            while base_angle > -100 :
                if req_exit:
                    return None

                # Get data
                data = camera.get_all_poses()

                # print data

                # Search all tags:
                if tag_index == -1:
                    for x in range(0,3):
                        if data[x] != [None, None]:
                            return data[x]
                else:
                    # Search for a specified tag
                    if data[tag_index] != [None, None]:
                        return data[tag_index]

                base_angle = base_angle - 5

                interface.send_absolute_angles(base_angle, 10, 10, 0)

                time.sleep(1)

        # change direction
        direction = direction * -1

    # user requested to exit
    interface.send_absolute_angles(10, 10, 10, 0)
    return None


def place_ducky(interface, target, joint_4_angle = 0):
    # PLACING THE DUCKY ON THE DESIRED TARGET
    # Get the position we want to place the ducky
    goal_xyz = get_xyz(interface, target) + np.array([[0], [0], [43]])

    # GETTING THE DUCKY
    # Move 30mm above the ducky
    ducky_xyz = DUCKY_POS + np.array([[0], [0], [30]])
    move_xyz(interface, ducky_xyz, True)
    time.sleep(1)
    # Move directly onto the ducky with pump on
    move_xyz(interface, DUCKY_POS, True)
    time.sleep(2)
    # Move to max height with the pump still on
    move_xyz(interface,np.reshape(np.array([149.66,-228.5,MAXHEIGHT]) ,(3, 1)), True)
    time.sleep(1)

    tmp = np.zeros((3, 1))
    tmp[:, :] = goal_xyz[:, :]
    tmp[2, 0] = MAXHEIGHT
    move_xyz(interface, tmp, True)
    time.sleep(1)

    # Move to desired position with pump on
    move_xyz(interface, goal_xyz, True)
    time.sleep(2)
    # Release the pump
    move_xyz(interface, goal_xyz, False)
    time.sleep(1)
    # Move back up
    move_xyz(interface, tmp, False)
    time.sleep(1)
    # Move to default position
    interface.send_absolute_angles(10, 10, 10, 0)


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
    search and place
    arm calibration
    touch test
    stack
    cleanup
    quit

    ENTER COMMAND:
    '''
    while True:
        print options
        command = raw_input("").lower()

        if command == "reset":
            # Reset angles to default positition
            interface.send_absolute_angles(0,10,10,0)

        elif command == "arm calibration":
            angles = interface.current_status.angles[0:3]
            # Get current XYZ
            P0t = DobotModel.forward_kinematics(angles)
            # Move down towards the table
            goal = [P0t[0],P0t[1], -20]
            move_xyz(interface, goal)
            time.sleep(5)
            # Move 150mm in the y-axis
            goal2 = np.reshape(goal ,(3, 1)) + np.reshape(np.array([0,150,0]) ,(3, 1))
            move_xyz(interface, goal2)


        elif command == "touch test":
            data = camera.get_all_poses()
            if data[0] != [None, None] and data[1] != [None, None] and data[2] != [None, None]:
                goal_1 = get_xyz(interface, data[0][0])
                goal_2 = get_xyz(interface, data[1][0])
                goal_3 = get_xyz(interface, data[2][0])
                move_xyz(interface, goal_1, False)
                time.sleep(1)
                interface.send_absolute_angles(0,10,10,0)
                time.sleep(1)
                move_xyz(interface, goal_2, False)
                time.sleep(1)
                interface.send_absolute_angles(0,10,10,0)
                time.sleep(1)
                move_xyz(interface, goal_3, False)
                time.sleep(1)
                interface.send_absolute_angles(0,10,10,0)

        elif command == "stack":
                data = camera.get_all_poses()
                if data[0] != [None, None] and data[1] != [None, None] and data[2] != [None, None]:
                    drop_location_1 = np.reshape(get_xyz(interface, data[0][0]) ,(3, 1)) + np.reshape(np.array([0,0,10]) ,(3, 1))
                    drop_location_2 = np.reshape(get_xyz(interface, data[0][0]) ,(3, 1)) + np.reshape(np.array([0,0,20]) ,(3, 1))

                    pickup_1 = get_xyz(interface, data[1][0])
                    pickup_2 = get_xyz(interface, data[2][0])

                    move_xyz(interface, pickup_1, True)
                    time.sleep(1)

                    tmp = np.zeros((3, 1))
                    tmp[:, :] = pickup_1[:, :]
                    tmp[2, 0] = 0
                    move_xyz(interface, tmp, True)
                    time.sleep(1)


                    move_xyz(interface, drop_location_1, True)
                    time.sleep(1)
                    move_xyz(interface, drop_location_1, False)
                    time.sleep(1)

                    tmp = np.zeros((3, 1))
                    tmp[:, :] = drop_location_1[:, :]
                    tmp[2, 0] = 0
                    move_xyz(interface, tmp, True)
                    time.sleep(1)

                    tmp = np.zeros((3, 1))
                    tmp[:, :] = pickup_2[:, :]
                    tmp[2, 0] = 0
                    move_xyz(interface, tmp, True)
                    time.sleep(1)


                    move_xyz(interface, pickup_2, True)
                    time.sleep(1)

                    tmp = np.zeros((3, 1))
                    tmp[:, :] = pickup_2[:, :]
                    tmp[2, 0] = 0
                    move_xyz(interface, tmp, True)
                    time.sleep(1)

                    move_xyz(interface, drop_location_2, True)
                    time.sleep(1)
                    move_xyz(interface, drop_location_2, False)

                    interface.send_absolute_angles(0,10,10,0)


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
            # Move 30mm above the ducky
            ducky_xyz = DUCKY_POS + np.array([[0], [0], [30]])
            move_xyz(interface, ducky_xyz, True)
            time.sleep(1)
            # Move directly onto the ducky with pump on
            move_xyz(interface, DUCKY_POS, True)
            time.sleep(2)
            # Move back up 30mm with the pump still on
            move_xyz(interface, ducky_xyz, True)
            time.sleep(1)
            # Move to default starting position
            interface.send_absolute_angles(0,10,10,0, interface.MOVE_MODE_JOINTS, True)

        elif command == "place ducky":
            # Which object to place ducky on?
            print object_selection
            print "Which object to place ducky on?"
            selection = int(input(""))
            # Get AR tag position
            data = camera.get_all_poses()[selection - 1]
            target = data[0]
            angle = data[1] # NEED TO UPDATE THIS
            np.reshape(target, (3, 1))
            if target != None:
                # Place the ducky on the target
                place_ducky(interface,target, 0) # 0 NEEDS TO BE CORRECT ANGLE
            else:
                print "Object is not available."

        elif command == "touch":
            # Which object to touch?
            print object_selection
            print "Which object to touch?"
            selection = int(input(""))
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

        elif command == "search and place":
            # Which object to search for?
            print object_selection
            print "Which object to search for?"
            selection = int(input(""))
            target = None
            # search until tag is found (Loop ensures that search is reactivated if tag is lost)
            while target is None:
                search(interface, camera, selection - 1)
                time.sleep(.5) # Wait a bit in case the detected object was moving

                # Get AR tag position
                data = camera.get_all_poses()[selection - 1]
                target = data[0]
                angle = data[1] # THIS NEEDS TO BE CORRECTED
                if target != None:
                    # Place the ducky on the target
                    place_ducky(interface,target, 0) # 0 NEEDS TO BE ANGLE
                    time.sleep(1)
                    interface.send_absolute_angles(0,10,10,0)
                    break;



        elif command == "quit":
            # Reset to default position
            interface.send_absolute_angles(0,10,10,0)
            # Stop the camera device
            sys.stdout.write("Releasing camera...")
            camera.release()
            print "OK!"
            break;
