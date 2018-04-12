from __future__ import print_function
import sys
import numpy as np
from math3D import *
from pyquaternion import Quaternion
from copy import deepcopy, copy

nrOfLinks = 6

class DH_table:
    alpha=np.zeros(nrOfLinks)
    a=np.zeros(nrOfLinks)
    d=np.zeros(nrOfLinks)
    theta_0=np.zeros(nrOfLinks)

class frame:
    def __init__(self):
        self.H=np.eye(4)
        self.rH=np.eye(4)

class link:
    def __init__(self):
        self.frameA=frame()
        self.frameB=frame()

class robot_kinematics:
    def __init__(self):
        self.dh_table=DH_table()
        self.Links =[link() for i in range(nrOfLinks+1)]
        self.theta = np.zeros(nrOfLinks)
        self.theta_home = np.zeros(nrOfLinks)
        IRB120_dh = DH_table()
        IRB120_dh.alpha =   np.array( [0, -90,   0, -90, 90, -90] )
        IRB120_dh.a =       np.array( [0,   0, 270,  70,  0,   0] )
        IRB120_dh.d =       np.array( [290, 0,   0, 302,  0, 72] )
        IRB120_dh.theta_0 = np.array( [0, -90,   0,   0,  0, 180] )
        self.setDH(IRB120_dh)
        self.goHome()

    def setDH(self, _dh_table):
        _dh_table.alpha= _dh_table.alpha * np.pi / 180
        _dh_table.theta_0= _dh_table.theta_0 * np.pi / 180
        self.dh_table= _dh_table

    def setJoints(self, _theta):
        _theta= _theta * np.pi / 180
        self.theta=deepcopy(_theta)
        self.solveFK()

    def goHome(self):
        self.theta = deepcopy(self.theta_home)
        self.solveFK()

    def solveFK(self):
        for i in range(1,7):
            H_alpha = np.eye(4)
            H_alpha[0:3,0:3] = AngleAxis(self.dh_table.alpha[i - 1], np.matrix('1;0;0'))
            H_a = np.eye(4)
            H_a[0,3] = self.dh_table.a[i - 1]
            H_theta = np.eye(4)
            H_theta[0:3,0:3]= AngleAxis(self.dh_table.theta_0[i - 1] + self.theta[i - 1], np.matrix('0;0;1'))
            H_d = np.eye(4)
            H_d[2, 3] = self.dh_table.d[i - 1]
            self.Links[i].frameB.H = deepcopy(self.Links[i - 1].frameB.H.dot(H_alpha.dot(H_a).dot(H_theta).dot(H_d)))

    def get_link_H(self,i):
        return self.Links[i].frameB.H

    def get_link_R(self,i):
        H = self.Links[i].frameB.H
        R = H[0:3,0:3]
        return R

    def get_link_p(self, i):
        H = self.Links[i].frameB.H
        p = H[0:3,3]
        return p

    def get_link_q(self, i):
        H = self.Links[i].frameB.H
        Q=Quaternion(matrix=H)
        return Q.elements

    def FK(self, _theta):
        self.setJoints(_theta)
        return self.Links[nrOfLinks].frameB.H

    def get_xyz(self):
        H = self.Links[nrOfLinks].frameB.H
        return H[0:3,3]

    def get_q(self, i):
        H = self.Links[nrOfLinks].frameB.H
        Q=Quaternion(matrix=H)
        return Q.elements

def ABB120_FK(J):
    robot=robot_kinematics()
    IRB120_dh = DH_table()
    IRB120_dh.alpha =   np.array( [0, -90,   0, -90, 90, -90] )
    IRB120_dh.a =       np.array( [0,   0, 270,  70,  0,   0] )
    IRB120_dh.d =       np.array( [290, 0,   0, 302,  0, 72] )
    IRB120_dh.theta_0 = np.array( [0, -90,   0,   0,  0, 180] )
    robot.setDH(IRB120_dh)
   
    return robot.FK(J)



from math import sin, cos, atan2, pi

def R3_6(H, alpha, theta, a, d):

    H01 = np.array(  [[cos(theta[0])                ,               - sin(theta[0]),              0 ,                   a[0]],
                      [cos(alpha[0]) * sin(theta[0]), cos(alpha[0]) * cos(theta[0]), - sin(alpha[0]), - d[0] * sin(alpha[0])],
                      [sin(alpha[0]) * sin(theta[0]), sin(alpha[0]) * cos(theta[0]),   cos(alpha[0]),   d[0] * cos(alpha[0])],
                      [                            0,                             0,               0,                      1]])

    H12 = np.array( [[cos(theta[1])                ,                - sin(theta[1]),                0,                   a[1]],
                     [cos(alpha[1]) * sin(theta[1]),  cos(alpha[1]) * cos(theta[1]),  - sin(alpha[1]), - d[1] * sin(alpha[1])],
                     [sin(alpha[1]) * sin(theta[1]),  sin(alpha[1]) * cos(theta[1]),    cos(alpha[1]),   d[1] * cos(alpha[1])],
                     [                            0,                              0,                0,                      1]])

    H23 = np.array( [[cos(theta[2])                ,                - sin(theta[2]),               0,                  a[2]],
                     [cos(alpha[2]) * sin(theta[2]),  cos(alpha[2]) * cos(theta[2]), - sin(alpha[2]), -d[2] * sin(alpha[2])],
                     [sin(alpha[2]) * sin(theta[2]),  sin(alpha[2]) * cos(theta[2]),   cos(alpha[2]),  d[2] * cos(alpha[2])],
                     [                            0,                              0,               0,                     1]])

    H0_3 = H01 @ H12 @ H23
    H3_6 = np.linalg.inv(H0_3) @ H

    R = H3_6[0:3, 0:3]
    return R


# ************** INVERS KINEMATICS  ************************************
## ONLY THE FIRST SOLUTION IS CORRECT ##

def ABB120_IK(H):
    solutions=[]
    q_min=[-155, -100, -85, -150, -110, -170]
    q_max=[ 155,  100,  65,  150,  110,  170]

    alpha_1 = 0 
    alpha_2 =-90 
    alpha_3 = 0
    alpha_4 =-90
    alpha_5 = 90 
    alpha_6 =-90 
    
    a1 = 0 
    a2 = 0 
    a3 = 270 
    a4 = 70 
    a5 = 0 
    a6 = 0 
    
    d1 = 290 
    d2 = 0 
    d3 = 0 
    d4 = 302
    d5 = 0 
    d6 = 72
    
    alpha=np.array([[alpha_1], [alpha_2], [alpha_3], [alpha_4], [alpha_5], [alpha_6]])*pi/180
    a=np.array([[a1], [a2], [a3], [a4], [a5], [a6]])
    d=np.array([[d1], [d2], [d3], [d4], [d5], [d6]])
    
    ax=H[0,2]
    ay=H[1,2]
    az=H[2,2]
    px=H[0,3]
    py=H[1,3]
    pz=H[2,3]
    
    
    px4=px-d6*ax
    py4=py-d6*ay
    pz4=pz-d6*az
    
    for i in range(1,3):
        if i==1:
            q1=atan2(py4,px4)
        else:
            q1=atan2(-py4,-px4)
        c1=cos(q1)
        s1=sin(q1)
        
        A=(pz4-d1)**2+(c1*px4+s1*py4-a2)**2-a3**2-d4**2-a4**2
        B=2*a3*d4
        C=2*a4*a3

        for j in range(1,2):
            check=0
            if j==1:
                if C==0:
                    s3=-A/B
                    q3=atan2(s3,(1-s3**2)**0.5)
                else:
                    if (A**2*B**2/C**4+(1+B**2/C**2)*(1-A**2/C**2))<0:
                            check=1
                    else:
                        s3=((-A*B/C**2+(A**2*B**2/C**4+(1+B**2/C**2)*(1-A**2/C**2))**0.5) / (1+B**2/C**2))
                        if abs(s3)>1 or abs((A+B*s3)/C)>1:
                            check=1
                        q3=atan2(s3,(A+B*s3)/C)
            else:
                if C==0:
                    s3=-A/B
                    q3=atan2(s3,-(1-s3**2)**0.5)
                else:
                    if (A**2*B**2/C**4+(1+B**2/C**2)*(1-A**2/C**2))<0:
                        check=1
                    else:
                        s3=((-A*B/C**2-(A**2*B**2/C**4+(1+B**2/C**2)*(1-A**2/C**2))**0.5) / (1+B**2/C**2))
                        if abs(s3)>1 or abs((A+B*s3)/C)>1:
                            check=1
                        q3=atan2(s3,(A+B*s3)/C)
            if check==0:
                c3=(A+B*s3)/C
                
                k1=c1*px4+s1*py4-a2
                k2=pz4-d1
                K=np.array([[k1, k2], [-k2, k1]])
                
                temp=np.linalg.inv(K) @ np.array([[a3-d4*s3+a4*c3], [d4*c3+a4*s3]])
                s2=temp[0]
                c2=temp[1]
                q2=atan2(s2,c2)
                
                #******************************************************************
                theta_1 = q1 
                theta_2 = q2 -pi/2
                theta_3 = q3 
                theta_4 = 0 
                theta_5 = 0 
                theta_6 = 0 +pi
                
                theta=np.array([[theta_1], [theta_2], [theta_3], [theta_4], [theta_5], [theta_6]])
                R=R3_6(H,alpha,theta,a,d)

                #******************************************************************
                
                r23=R[1,2]
                r13=R[0,2]
                r21=R[1,0]
                r22=R[1,1]
                r33=R[2,2]
                
                for k in range(1,3):
                    if k==1:
                        q5=atan2((1-r23**2)**0.5,r23)
                    else:
                        q5=atan2(-(1-r23**2)**0.5,r23)

                    s5=sin(q5)
                    
                    q4=atan2(r33/s5,-r13/s5)
                    q6=atan2(r22/s5,-r21/s5)
                    q=[q1*180/pi, q2*180/pi, q3*180/pi, q4*180/pi, q5*180/pi, q6*180/pi]
                    if check==0:
                        out_range=False
                        for ii in range(6):
                            if q[ii]>q_max[ii] or q[ii]<q_min[ii]:
                                out_range=True
                        if not(out_range):
                            solutions.append(q)
    if len(solutions)==0:
        solutions=None

    return solutions
    


