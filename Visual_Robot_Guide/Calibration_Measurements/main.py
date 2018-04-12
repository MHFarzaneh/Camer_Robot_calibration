import sys
import os
from time import sleep
import numpy as np
sys.path.append('../ConnectABB/')
sys.path.append('../Kinematics/')
from ABB_Socket import ABB_Socket
from robot_kinematics import ABB120_FK, ABB120_IK

joints_list=np.loadtxt('Input/joints_list.txt')

input("Please run the rapid file then press enter >>")
connection = ABB_Socket("192.168.125.1", 1025)
connection.SetTCPSpeed(50)

connection.MoveAbsJ([10, 20, 30, 0, -10, 20])
J=connection.ReadJoints()
print(J)
J=np.array(J)
H=ABB120_FK(J)
J_new=ABB120_IK(H)
print(H)
print(J_new)












'''
measured_joints=[]
measured_cam=[]
for i in range(len(joints_list)):
    connection.MoveAbsJ(joints_list[i])
    print('Measurement number: ', i)
    measured_joints.append(connection.ReadJoints())


    np.savetxt('Output/measured_joints.txt', measured_joints)
'''

connection.delete()

