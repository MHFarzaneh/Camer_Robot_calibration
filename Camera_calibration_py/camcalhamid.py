import numpy as np
import cv2
import glob

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
   

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
        # SPACE pressed
        #img_name = "opencv_frame_{}.png".format(img_counter)
        #images.append(frame)
        #cv2.imwrite(img_name, frame)
        #print("{} written!".format(img_name))
        #img_counter += 1
        #img = cv2.imread(fname)
        #cv2.imshow('img',frame)
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
			
			



while True:
    ret, frame = cam.read()
    h,  w = frame.shape[:2]
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
        cv2.imshow('test',dst)
        #k = cv2.waitKey(0) & 0xff
        #if k == 's':
           # cv2.imwrite(fname[:6]+'.png', img)
			
    cv2.imshow("test", dst)
    if not ret:
        break
		
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
cam.release()
cv2.destroyAllWindows()


#images = glob.glob('*.png')
#print(images)

