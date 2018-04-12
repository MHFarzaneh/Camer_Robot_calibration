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
#print(J)
J=np.array(J)
H=ABB120_FK(J)
J_new=ABB120_IK(H)
#print(H)
#print(J_new)

xyz = cam1.getXYZ()
print(xyz)
print("-------------")


connection.MoveAbsJ([-60, 0, 0, 0, 90, 45])
J=connection.ReadJoints()
#print(J)
J=np.array(J)
H=ABB120_FK(J)
J_new=ABB120_IK(H)
#print(H)
#print(J_new)
xyz = cam1.getXYZ()
print(xyz)
print("-------------")


connection.MoveAbsJ([-65, 0, 0, 0, 90, 45])
J=connection.ReadJoints()
#print(J)
J=np.array(J)
H=ABB120_FK(J)
J_new=ABB120_IK(H)
#print(H)
#print(J_new)
xyz = cam1.getXYZ()
print(xyz)
print("-------------")



input()
cam1.stop()

connection.delete()

