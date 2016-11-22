import cv2
import argparse
import numpy as np
from threading import Thread
import time
from hampy import detect_markers
import warnings

class Camera:
    def __init__(self, camera_id = -1, ducky = [None, None] ,duckybot = [None, None], obstacle = [None, None]):
        warnings.simplefilter("ignore")
        
        self.ID = camera_id
        self.ducky_tag = ducky
        self.duckybot_tag = duckybot
        self.obstacle_tag = obstacle

        # Intrinsic Parameters
        self.camMatrix = np.zeros((3, 3),dtype=np.float64 )
        self.camMatrix[0][0] = 810.06435
        self.camMatrix[0][2] = 325.39548
        self.camMatrix[1][1] = 810.75645
        self.camMatrix[1][2] = 249.01798
        self.camMatrix[2][2] = 1.0    
        
        self.distCoeff = np.zeros((1, 5), dtype=np.float64)
        self.distCoeff[0][0] = 0.01584
        self.distCoeff[0][1] = 0.37926
        self.distCoeff[0][2] = -0.00056
        self.distCoeff[0][3] = 0.00331
        self.distCoeff[0][4] = 0.0        
        
        # video feed is OFF (default)
        self.active_video = False
        
        
    def initialize(self):
        # Initialize video capture
        self.cap = cv2.VideoCapture(self.ID)
        
        if cv2.__version__[0] == "2":
            # Latest Stable Version
            self.cap.set(cv2.cv.CV_CAP_PROP_FPS, 10.0)
            frameRate = self.cap.get(cv2.cv.CV_CAP_PROP_FPS)
            frameWidth = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
            frameHeight = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
        else:
            # version 3.1.0 (Dylans Version)
            self.cap.set(cv2.CAP_PROP_FPS, 10.0)
            frameRate = self.cap.get(cv2.CAP_PROP_FPS)
            frameWidth = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            frameHeight = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)         

        self.thresh = 0.4        
        self.thresh_img = np.zeros((frameHeight, frameWidth, 3), dtype=np.uint8)
        kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]], dtype=np.float32)
        
    def release(self):
        # Release video capture
        self.cap.release()


    def __str__(self):
        # String representation of this camera
        output = "===========CAMERA INFORMATION===========\n"
        output += "Camera Device ID: " + str(self.ID)
        output += "\n\nIntrinsic Parameters: \n" + str(self.camMatrix) + "\n"
        output += "\nRegistered AR Tags:"
        if self.ducky_tag != [None, None]:
            output += "\nDucky:    \t ID: {:10} \t Size: {:3}mm".format(self.ducky_tag[0], self.ducky_tag[1])
        if self.duckybot_tag != [None, None]:
            output += "\nDuckybot: \t ID: {:10} \t Size: {:3}mm".format(self.duckybot_tag[0], self.duckybot_tag[1])        
        if self.obstacle_tag != [None, None]:
            output += "\nObstacle: \t ID: {:10} \t Size: {:3}mm".format(self.obstacle_tag[0], self.obstacle_tag[1])
        output += "\n========================================\n"
        return output
            
    def activate_video(self):
        # Activate Video Feed
        self.active_video = True
        self.video_thread = Thread(target=self.start_video)
        self.video_thread.start()  
        
    def deactivate_video(self):
        # Deactivate Video Feed
        self.active_video = False
        # wait for video thread to finish
        while self.video_thread.is_alive():
            time.sleep(.1)
        cv2.destroyAllWindows()


    def start_video(self):
        while self.active_video:
            # Get Data
            (ducky, duckybot, obstacle, img) = self.capture_data()    

            # Show Video
            cv2.imshow('live', img)
            if cv2.waitKey(1) & 0xFF == ord('p'):
                cv2.imwrite('out.jpg', img)
        
        
    def capture_data(self):
        # Initial Values
        Ducky_Pose = [None, None] 
        Duckybot_Pose = [None, None]   
        Obstacle_Pose = [None, None]   
        
        # Get Frame
        okay, img = self.cap.read()
        if not okay:
            return [Ducky_Pose, Duckybot_Pose, Obstacle_Pose, img]
        
        # convert image to grayscale then back so it still has
        # 3 color dimensions
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)

        # threshold the image to either pure white or pure black
        # thresh is scaled so that images is thresholded at % out of 255
        self.thresh_img[:, :, :] = 255.0*np.round(gray_img[:, :, :]/(510.0*self.thresh))

        # blur the rounded image
        blurred_image = cv2.GaussianBlur(self.thresh_img,(5,5),0)
        
        # find any valid AR tags in the image
        markers = detect_markers(self.thresh_img)

        # for each valid tag, get the pose
        for m in markers:
            # Draw the marker outline on the image
            m.draw_contour(img)

            # Label the tag ID on the image
            if cv2.__version__[0] == "2":
                # Latest Stable Version
                cv2.putText(img, str(m.id), tuple(int(p) for p in m.center), cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            else:
                # version 3.1.0 (Dylans Version)
                cv2.putText(img, str(m.id), tuple(int(p) for p in m.center), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)            

            # Get ducky pose if ducky AR tag was detected
            if (m.id == self.ducky_tag[0]):
                Ducky_Pose = self.get_object_pose(m, self.ducky_tag)

            # Get duckybot pose if duckybot AR tag was detected
            elif (m.id == self.duckybot_tag[0]):
                Duckybot_Pose = self.get_object_pose(m, self.duckybot_tag)
                
            # Get obstacle pose if obstacle AR tag was detected
            elif (m.id == self.obstacle_tag[0]):
                Obstacle_Pose = self.get_object_pose(m, self.obstacle_tag)
                          
        
        # Return all poses and the source image
        return [Ducky_Pose, Duckybot_Pose, Obstacle_Pose, img]

    
    # Given a matching marker and tag, get the pose
    def get_object_pose(self, marker, tag):
        # AR Tag Dimensions
        objPoints = np.zeros((4, 3), dtype=np.float64)
        objPoints[0,0] = -1.0*tag[1]/2.0
        objPoints[0,1] = tag[1]/2.0
        # objPoints[0,2] = 0.0
        objPoints[1,0] = tag[1]/2.0
        objPoints[1,1] = tag[1]/2.0
        # objPoints[1,2] = 0.0
        objPoints[2,0] = tag[1]/2.0
        objPoints[2,1] = -1*tag[1]/2.0
        # objPoints[2,2] = 0.0
        objPoints[3,0] = -1*tag[1]/2.0
        objPoints[3,1] = -1*tag[1]/2.0
        # objPoints[3,2] = 0.0

        # Get each corner of the tags
        imgPoints = np.zeros((4, 2), dtype=np.float64)        
        for i in range(4):
            imgPoints[i, :] = marker.contours[i, 0, :]


        camPos = np.zeros((3, 1))
        camRot = np.zeros((3, 1))             
        
        # SolvePnP
        retVal, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camMatrix, self.distCoeff)
        Roa, b = cv2.Rodrigues(rvec)
        Pca = tvec
        
        return [Pca, Roa]        
    
    
