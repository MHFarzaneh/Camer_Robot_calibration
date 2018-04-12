import numpy as np
import cv2
import glob
import pickle


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    #print(corner)
    #cv2.waitKey(3000)
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
    return img
   

def AugFrame(calibparam):
    # Getting back the objects:
    with open(calibparam, 'rb') as f:  # Python 3: open(..., 'rb')
        ret, mtx, dist, rvecs, tvecs = pickle.load(f)
		
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
    #cam.set(3,1280)
    #cam.set(4,720)

    cv2.namedWindow("test")
	
    
    while True:
        ret, frame = cam.read()
        h,  w = frame.shape[:2]
		
        cv2.line(frame,(0 ,int(h/2) ), (w ,int(h/2)), (100,100,100), 2)
        cv2.line(frame,(int(w/2),0 ), (int(w/2),h), (100,100,100), 2)
		
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        # undistort
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        #draw 3D frame
        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
	
        gray = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
        ret2, corners = cv2.findChessboardCorners(gray, (9,6),None)

        if ret2 == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            # Find the rotation and translation vectors.
            _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            dst = draw(dst,corners2,imgpts)
            dst = cv2.flip( dst, -1 )
            cv2.imshow('test',dst)
			
            k = cv2.waitKey(1)
			
            if k%256 == 32:
                print("*****************************")
                #print("imgpts: \n{}".format(imgpts))
                print("rvecs: \n{}".format(rvecs))
                print("tvecs: \n{}".format(tvecs))
                rodr, J = cv2.Rodrigues(rvecs)
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
        else:
            dst = cv2.flip( dst, -1 )
            cv2.imshow('test',dst)		
        
        
        #cv2.imshow("test", dst)
        
        if not ret:
            break
		
        k = cv2.waitKey(1)

        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
			
    cam.release()
    cv2.destroyAllWindows()