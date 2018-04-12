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
cam1 = CamThread(1, "First")

# Start new Threads
cam1.start()
time.sleep(6)
print('try to stop')
#Cam1.join(1)
cam1.stop()
print('stopped')