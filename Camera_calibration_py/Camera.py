import numpy as np
import cv2
import glob
import pickle
import threading 
import time

class CamThread (threading.Thread):
    def __init__(self, name,desiredW=640,desiredH=480):
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.name = name
        self.desiredH=desiredH
        self.desiredW=desiredW
        self.isCamReady = 0

    def go(self):
        self.start()
        while self.isCamReady!=1:
            time.sleep(0.001)


    def stop(self):
        self.exitFlag = 0

    def run(self):
        print ("Starting " + self.name)
        
        # starts camera 
        self.cam = cv2.VideoCapture(0)
        
        #Set resolution
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT,self.desiredH)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH ,self.desiredW)

        #Make a window
        self.window = cv2.namedWindow(self.name)
        self.readret, self.frame = self.cam.read()
        cv2.imshow(self.name,self.frame)
        self.height,  self.width = self.frame.shape[:2]
        print(self.width)
        print(self.height)
        self.k = cv2.waitKey(1)

        self.exitFlag = 1            #Still do the thread loop
        self.iscalibrated = 0        #Calibrated parameters has been loaded/ is camera manually calibrated
        self.isAR = 0                #Is in the augmented reality mode
        self.isRainbow = 0           #To show the rain of detected corners
        self.isFlip = 0              #To flip the image
        self.isCamReady = 1          #Is the camera ready

        while self.exitFlag:
            self.readret, self.frame = self.cam.read()
            self.isRainbow = 0
            self.k = cv2.waitKey(1)
            if self.iscalibrated == 0 and self.isAR == 0:
                cv2.imshow(self.name,self.frame)
                self.k = cv2.waitKey(1)
                if self.isRainbow == 1:self.k = cv2.waitKey(100)
            elif self.iscalibrated == 1 and self.isAR == 0:
                if self.isFlip == 1: self.frame = cv2.flip( self.frame, -1 )
                # undistort
                self.frame = cv2.undistort(self.frame, self.mtx, self.dist, None, self.newcameramtx)
                cv2.imshow(self.name,self.frame)
                self.k = cv2.waitKey(1)
            elif self.iscalibrated == 1 and self.isAR == 1:
                if self.isFlip == 1: self.frame = cv2.flip( self.frame, -1 )
                # undistort
                self.frame = cv2.undistort(self.frame, self.mtx, self.dist, None, self.newcameramtx)
                self.augmentedReality()
                cv2.imshow(self.name,self.frame)
                self.k = cv2.waitKey(1)

            if self.k%256 == 27:
                # ESC pressed
                print("Escape hit, closing...")
                self.stop()
                print("*******")
        

        #self.cam.release()
        print ("Exiting " + self.name)


    def calibrate(self,filename='empty'):
        if filename != 'empty':
            # Getting back the objects:
            with open(filename, 'rb') as f:  # Python 3: open(..., 'rb')
                self.retcalibrated, self.mtx, self.dist, self.rvecs, self.tvecs = pickle.load(f)
            print("Camera calibrated by file.")
        
        
        self.iscalibrated = 1
        
        
        #Prepare undistored matrix
        self.newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(self.width, self.height),1,(self.width, self.height))


    def usrCalibrate(self,isFlip=0):
        self.isFlip = isFlip
        print("Use 6 by 9 (corners) checker board")
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        img_counter = 0
        
        print("Press SPACE to take a pic, press ESC to finish")
        
        while True:
            if not self.readret:
                break
            
            #e = cv2.waitKey(1)
            #print(e)

            if self.k%256 == 27:
                # ESC pressed
                print("Escape hit, calibration done.")
                break
            elif self.k%256 == 32:
                print("here")
                gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
                print("image read")
            
                # Find the chess board corners
                retcorners, corners = cv2.findChessboardCorners(gray, (9,6),None)
            
                # If found, add object points, image points (after refining them)
                if retcorners == True:
                    objpoints.append(objp)
                    print("Chess board deteceted")

                    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                    imgpoints.append(corners2)
                
                    # Draw and display the corners
                    self.frame = cv2.drawChessboardCorners(self.frame, (9,6), corners2,self.readret)
                    cv2.imshow(self.name,self.frame)
                    self.isRainbow = 1
                    self.k = cv2.waitKey(100)
                    
                    
        self.retcalibrated, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        print("Camera calibrated by user.")
        self.iscalibrated = 1
        mean_error = 0

        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], self.rvecs[i], self.tvecs[i], self.mtx, self.dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error

        print("total error: ", mean_error/len(objpoints))		

    
	
    def saveCalibParam(self,filename):
        # Saving the objects:
        with open(filename, 'wb') as f:  # Python 3: open(..., 'wb')
            pickle.dump([self.retcalibrated, self.mtx, self.dist, self.rvecs, self.tvecs], f)

    def drawFrame(self,img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        #print(corner)
        #cv2.waitKey(3000)
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 3)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
        return img


        
		


    def startAR(self,isFlip=0):

        self.isFlip = isFlip
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((6*9,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

        #draw 3D frame
        self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

        self.isAR = 1

    def augmentedReality(self):

        #Add crosshead
        cv2.line(self.frame,(0 ,int(self.height/2) ), (self.width ,int(self.height/2)), (100,100,100), 2)
        cv2.line(self.frame,(int(self.width/2),0 ), (int(self.width/2),self.height), (100,100,100), 2)


    
        gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)
        retcorners, corners = cv2.findChessboardCorners(gray, (9,6),None)

        if retcorners == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)

            # Find the rotation and translation vectors.
            _, self.rvecs, self.tvecs, inliers = cv2.solvePnPRansac(self.objp, corners2, self.mtx, self.dist)

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(self.axis, self.rvecs, self.tvecs, self.mtx, self.dist)

            self.frame = self.drawFrame(self.frame,corners2,imgpts)
            #cv2.imshow(self.name,self.frame)
            #cv2.waitKey(1)
            
            if self.k%256 == 32:
                print("*****************************")
                #print("imgpts: \n{}".format(imgpts))
                print("rvecs: \n{}".format(self.rvecs))
                print("tvecs: \n{}".format(self.tvecs))
                rodr, J = cv2.Rodrigues(self.rvecs)
                print("Rodrigues: \n{}".format(rodr))				
                #print("mtx: \n{}".format(mtx))
                #print("dist: \n{}".format(dist))
                #print("axis: \n{}".format(axis))
                #print("objp: \n{}".format(objp))
                print("*****************************")
                cv2.waitKey(3000)   
            #k = cv2.waitKey(0) & 0xff
            #if k == 's':
            # cv2.imwrite(fname[:6]+'.png', img)
        # else:
        #     dst = cv2.flip( dst, -1 )
        #     cv2.imshow('test',dst)		
        
        
        #cv2.imshow("test", dst)
        
        # if not ret:
        #     break
        
        # k = cv2.waitKey(1)

        # if k%256 == 27:
        #     # ESC pressed
        #     print("Escape hit, closing...")
        #     break
		
    def getXYZ(self):
        return np.array(self.tvecs)*25.4


# class Pattern:
#     def __init__(self):



