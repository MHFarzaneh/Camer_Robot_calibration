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
from kaveh_pso import pso




Measure_list,J_list = loadCalibData('Measure_list_file','J_list_file')



# with open('Measure_list_file', 'rb') as f:  # Python 3: open(..., 'wb')
#     Measure_list= pickle.load(f)


# with open('J_list_file', 'rb') as f:  # Python 3: open(..., 'wb')
#     J_list = pickle.load(f)

print("-------------")
print("-------------")

ml,jl = loadCalibData('Measure_list_file','J_list_file')

print(Measure_list)
print("-------------")
print(J_list)

print("-------------")
print("-------------")

v = (np.ones((1,12))*10).tolist()[0]
print(v)

print(loss(v))

LB=[-2000, -2000, -2000, -180, -180, -180, -300, -300, -300 , -300, -300, -300]
UB=[+4000, +4000, +4000, +180, +180, +180, +400, +400, +400, +400, +400, +400]

PSO = pso(loss, 12, LB, UB, Max_It=4000,n_Pop = 20,phi_1 = 2.05,phi_2 = 2.05)
PSO.run()




