from Camera import *
import time
import sys
import os
sys.path.append('..\Visual_Robot_Guide\ConnectABB')
sys.path.append('..\Visual_Robot_Guide\Kinematics')
from ABB_Socket import ABB_Socket
from robot_kinematics import ABB120_FK, ABB120_IK
import numpy as np
from calibModel import *
import pickle


J_list=[]
Measure_list=[]

# Create new threads
cam1 = CamThread("Camera")

# Start new Threads
cam1.start()
time.sleep(1)
print("-------------")
cam1.calibrate("Calib_param.pkl")
cam1.startAR()

time.sleep(1)
input("Please run the rapid file then press enter >>")
connection = ABB_Socket("192.168.125.1", 1025)
connection.SetTCPSpeed(50)

connection.MoveAbsJ([-55, 0, 0, 0, 90, 45])
J=connection.ReadJoints()
J=np.array(J)
H=ABB120_FK(J)
J_new=ABB120_IK(H)

time.sleep(1)
J_list.append(J)
xyz = cam1.getXYZ()
Measure_list.append(xyz)
print("-------------")


connection.MoveAbsJ([-60, 0, 0, 0, 90, 45])
J=connection.ReadJoints()
J=np.array(J)
H=ABB120_FK(J)
J_new=ABB120_IK(H)

time.sleep(1)
J_list.append(J)
xyz = cam1.getXYZ()
Measure_list.append(xyz)
print("-------------")


connection.MoveAbsJ([-65, 0, 0, 0, 90, 45])
J=connection.ReadJoints()
J=np.array(J)
H=ABB120_FK(J)
J_new=ABB120_IK(H)

time.sleep(1)
J_list.append(J)
xyz = cam1.getXYZ()
Measure_list.append(xyz)
print("-------------")

print(Measure_list)
print("-------------")
print(J_list)

# Saving the objects:
with open('Measure_list_file', 'wb') as f:  # Python 3: open(..., 'wb')
    pickle.dump(Measure_list, f)

with open('J_list_file', 'wb') as f:  # Python 3: open(..., 'wb')
    pickle.dump(J_list, f)

input()
cam1.stop()

connection.delete()

