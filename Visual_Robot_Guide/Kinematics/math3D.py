import numpy as np

def AngleAxis(angle,axis):
    x = axis[0,0]
    y = axis[1,0]
    z = axis[2,0]

    c=np.cos(angle)
    s=np.sin(angle)
    v=1-c

    R=np.matrix([[x*x*v+c,   x*y*v-z*s, x*z*v+y*s],
                 [x*y*v+z*s, y*y*v+c,   y*z*v-x*s],
                 [x*z*v-y*s, y*z*v+x*s, z*z*v+c  ]])
    return R

def R_EulerZYX(alpha,beta,gamma):
    Rz=AngleAxis(alpha,np.array([[0],[0],[1]]))
    Ry=AngleAxis(beta, np.array([[0],[1],[0]]))
    Rx=AngleAxis(gamma,np.array([[1],[0],[0]]))
    R=np.matmul(np.matmul(Rz,Ry),Rx)
    return R

def H_EulerZYX(x,y,z,alpha,beta,gamma):
    H=np.eye(4)
    H[0:3,3]=[x,y,z]
    H[0:3,0:3]=R_EulerZYX(alpha,beta,gamma)
    return H

