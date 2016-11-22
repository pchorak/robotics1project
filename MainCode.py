import AR_Camera
import time
import sys
import numpy as np
import DobotModel
import SerialInterface

#=====DEFINED CONSTANTS=====
CAMERA_ID = 0
DUCKY = [16287382, 51]
DUCKYBOT = [56356251, 25]
OBSTACLE = [425234, 51]



# Display video feed
def show_video(camera):
    print "Activating video..."
    camera.activate_video()



# Display poses of all objects
# [ [Vector tag to camera] ,  [Rotation of tag] ]
# If not present, [None, None] will be reported
def get_data(camera):
    (ducky, duckybot, obstacle, img) = camera.capture_data()
    print "----------------------------"
    print "Ducky Pose:", ducky
    print "Duckybot Pose:", duckybot
    print "Obstacle Pose:", obstacle
    print "----------------------------"



# Return the Dobot joint angles needed to go to desired XYZ
def get_angles(coordinates):
    return DobotModel.inverse_kinematics(coordinates)



# Move Dobot to a desired XYZ position
def move_xyz(interface, target):
    angles = get_angles(target)
    if any(np.isnan(angles)):
        print "Error: No solution for coordinates: ", target
    else:
        interface.send_absolute_angles(angles[0],angles[1],angles[2],0.0) 



# Touch the end effector to a detected object
def touch(interface, detected_object , mode):
    # Get current XYZ
    P0t = DobotModel.forward_kinematics(interface.current_status.get_angles())
    
    # If arial mode and z is low, move up a bit  
    if mode == 2 and P0t[3] < 0:
        P0t = P0t + [0,0,10]
        move_xyz(interface, P0t)

    # Getting Desired XYZ of end effector
    Pct = np.array([[-10.48], [-21.58], [28.42]])
    Roc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    Pta = np.matmul(Roc, detected_object[0]) - np.matmul(Roc, Pct)
    
    target = np.reshape(Pta, (3, 1)) + np.reshape(P0t, (3, 1))
    
    # DIRECT MODE
    if mode == 1:
        # Move directly to target
        move_xyz(interface, target)   
        
    # ARIAL MODE
    elif mode == 2:
        # Move directly above target
        move_xyz(interface, target + [0,0,10]) 
        # Move to target
        move_xyz(interface, target)   



# Track an AR Tag. DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
def track(interface, camera, tag_index):    
    
    while True:
        # Only enter search if capture_data failed to find tag for 10 consecutive frames
        searching = True
        for x in range(0,10):
            time.sleep(0.1)
            data = camera.capture_data()[tag_index]   
            if data != [None, None]:
                searching = False
                break;
        
        if searching:
            # Search until you find the desired tag
            data = search(interface, camera, tag_index)   
        
        # Follow tag while it is in view
        while data != [None, None]:
                # Getting Desired XYZ of end effector
                Pct = np.array([[0], [0], [130]])
                Roc = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
                Pta = np.matmul(Roc, data[0]) - np.matmul(Roc, Pct)
                
                # If the change in desired XYZ is small, don't move
                if np.linalg.norm(Pta) < 10:
                    # Get data
                    data = camera.capture_data()[tag_index]                   
                    continue
    
                p0t = DobotModel.forward_kinematics(interface.current_status.get_angles())
    
                target = np.reshape(Pta, (3, 1)) + np.reshape(p0t, (3, 1))
                # Move end effector to ducky
                angles = move_xyz(interface,target)
                time.sleep(0.5)
                
                # Get data
                data = camera.capture_data()[tag_index] 
    


# Search for AR Tag. DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
# If tag_index is left undefined , it will search until any registered tag is found
def search(interface, camera, tag_index = -1):
    
    # GO TO STARTING POSITION
    interface.send_absolute_angles(0, 10, 10, 0)
    
    while True:
        interface.send_jog_command(False,2,30)
        while interface.current_status.angles[0] > -130:
            # Get data
            data = camera.capture_data()
            # Search all tags
            if tag_index == -1:
                for x in range(0,3):
                    if data[x] != [None, None]:
                        return data[x]
                        interface.send_jog_command(False,0,0)
            else:
                # Search for a specified tag
                if data[tag_index] != [None, None]:
                    return data[tag_index]
                    interface.send_jog_command(False,0,0)
        interface.send_jog_command(False,1,30)
        while interface.current_status.angles[0] < 130:
            # Get data
            data = camera.capture_data()
            # Search all tags
            if tag_index == -1:
                for x in range(0,3):
                    if data[x] != [None, None]:
                        return data[x]
                        interface.send_jog_command(False,0,0)
            else:
                # Search for a specified tag
                if data[tag_index] != [None, None]:
                    return data[tag_index]
                    interface.send_jog_command(False,0,0)

if __name__ == '__main__':

    # INTITIALIZING SERIAL INTERFACE
    interface = SerialInterface.SerialInterface('/dev/ttyACM0') # Linux

    time.sleep(1)
    interface.send_absolute_angles(0,10,10,0)
    time.sleep(1)

    # INITIALIZING CAMERA
    sys.stdout.write("Initiliazing the Camera..." )
    camera = AR_Camera.Camera(CAMERA_ID, DUCKY, DUCKYBOT, OBSTACLE)
    camera.initialize()
    print "OK!\n"

    # DISPLAY CAMERA PROPERTIES
    print camera
    
    object_selection = '''OBJECTS:
    1. ducky
    2. duckybot
    3. obstacle
    '''    

    options = '''COMMANDS:
    show video
    get data
    touch
    track
    quit
    '''
    while True:
        print options
        command = raw_input("Enter Command => ").lower()

        if command == "show video":
            show_video(camera)
        elif command == "get data":
            get_data(camera)
        elif command == "touch":
            # Which object to touch?
            print object_selection
            selection = int(input("Which object to touch: "))
            # Get data
            data = camera.capture_data()[selection - 1]
            # Touch the object if it is available
            if data != [None,None]:
                touch(interface, data, 1)
            else:
                print "Object is not available."

        elif command == "track":
            # Which object to track?
            print object_selection
            selection = int(input("Which object to track: "))
            # Begin Tracking
            track(interface, camera, selection - 1)
            
        elif command == "quit":
            sys.stdout.write("Releasing camera...")
            camera.release()
            print "OK!"
            break;
