import numpy as np
import cv2
import glob
import pickle


def CamCalib():

    print("Use 6 by 9 checker board")
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # Take pictures
    cam = cv2.VideoCapture(0)

    cv2.namedWindow("test")

    img_counter = 0

    #images = glob.glob('*.png')
	
    print("Press SPACE to take a pic, press ESC to finish")
	
    while True:
        ret, frame = cam.read()
        cv2.imshow("test", frame)
        if not ret:
            break
		
        k = cv2.waitKey(1)

        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif k%256 == 32:
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            print("image read")
        
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
        
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                print("Chess board deteceted")

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)
            
                # Draw and display the corners
                frame = cv2.drawChessboardCorners(frame, (9,6), corners2,ret)
                cv2.imshow('test',frame)
                cv2.waitKey(100)
				
				
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    mean_error = 0

    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

    print("total error: ", mean_error/len(objpoints))		

    
	

    # Saving the objects:
    with open('Calib_param.pkl', 'wb') as f:  # Python 3: open(..., 'wb')
        pickle.dump([ret, mtx, dist, rvecs, tvecs], f)
		
    cam.release()
    cv2.destroyAllWindows()


