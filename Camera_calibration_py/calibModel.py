import sys
import os
sys.path.append('..\Visual_Robot_Guide\Kinematics')
from robot_kinematics import ABB120_FK
import numpy as np
from  math3D import *
import pickle

J_list=[]
Measure_list=[]

def loadCalibData(Measure_list_file,J_list_file):
    global J_list, Measure_list
    # Getting back the objects:
    with open(Measure_list_file, 'rb') as f:  # Python 3: open(..., 'rb')
        Measure_list = pickle.load(f)
    with open(J_list_file, 'rb') as f:  # Python 3: open(..., 'rb')
        J_list = pickle.load(f)
    print("Data  loaded.")

    return Measure_list,J_list

def model(J,v):
    H_cam_wrt_flang    = H_EulerZYX(v[0],v[1],v[2],v[3],v[4], v[5] )
    H_pattern_wrt_base = H_EulerZYX(v[6],v[7],v[8],v[9],v[10],v[11])
    H_pattern_wrt_cam = np.linalg.inv( np.matmul( np.linalg.inv( H_pattern_wrt_base ), np.matmul( ABB120_FK(J), H_cam_wrt_flang ) ) ) 
    X_pattern_wrt_cam = H_pattern_wrt_cam[0:3,3]
    return X_pattern_wrt_cam

def loss(v):
    global J_list, Measure_list
    error=0
    for i in range(len(J_list)):
        error= error+np.linalg.norm(Measure_list[i]-model(J_list[i],v))
    return error/len(J_list)
