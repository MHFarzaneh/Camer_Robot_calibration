from Camera import *
import time
import sys
import os
sys.path.append('..\Visual_Robot_Guide\ConnectABB')
sys.path.append('..\Visual_Robot_Guide\Kinematics')
from ABB_Socket import ABB_Socket
from robot_kinematics import ABB120_FK, ABB120_IK
import numpy as np




# Create new threads
cam1 = CamThread("Camera")

# Start new Threads
cam1.go()
#time.sleep(1)
print("-------------")
cam1.usrCalibrate()
cam1.saveCalibParam('Cam_calib_param_2018-04-12_HD.pkl')

cam1.calibrate("Cam_calib_param_2018-04-12_HD.pkl")


cam1.startAR(1)