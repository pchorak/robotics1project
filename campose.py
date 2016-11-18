# Determine the transformation between the end effector and arm-mounted camera
import numpy as np
import numpy.random as rng

import math3D
import DobotModel

def get_pose(angle_list,pca_list):
    """
    Input:
        angle_list - list of sets of joint angles [(a0,a1,a2)]
        pca_list - list of measured camera vectors [[3 x 1]]
    Output: (ptc,Rtc)
        ptc - estimated offset of the camera origin (c) from the end effector (t)
        Rtc - estimated rotation between the camera frame (c) and the end effector frame (t)
    """
    I = np.eye(3)
    b = []
    A = []
    # Form a large matrix of measurements
    for (angles,pca) in zip(angle_list,pca_list):
        Rt0 = np.transpose(DobotModel.R0T(angles))
        p0t = np.transpose(np.matrix(DobotModel.forward_kinematics(angles)))
        b.append(-Rt0*p0t)
        A.append(np.hstack((I,np.kron(I,np.transpose(pca)),-Rt0)))
    b = np.vstack(b)
    A = np.vstack(A)
    # Least-squares estimate (psuedo-inverse to solve for x)
    x = np.linalg.pinv(A)*b
    ptc = x[0:3]
    Rtc = nearest_rot(np.reshape(x[3:12],[3,3])) # enforce SO(3) properties
#    p0a = x[12:15]
    return (ptc,Rtc)

def nearest_rot(M):
    """
    Input: a 3x3 matrix M
    Output: the rotation matrix nearest to M
    """
    (U,_,V) = np.linalg.svd(M)
    if np.linalg.det(U*V) < 0:
        return U*np.diag([1,1,-1])*V
    else:
        return U*V

def test():
    n = 20 # number of simulated measurements

    # Hidden parameters
    p0a = rng.normal(0,150,[3,1]) # mm
    ptc = rng.normal(0,50,[3,1]) # mm
    Rtc = math3D.rot(rng.normal(0,1,3),rng.normal(0,90))

    # Simulate measurements
    angle_list = []
    pca_list = []
    for k in range(n):
        angles = tuple(rng.normal(0,1,3))
        # Dobot kinematics
        Rt0 = np.transpose(DobotModel.R0T(angles))
        p0t = np.transpose(np.matrix(DobotModel.forward_kinematics(angles)))
        # calculate camera vector
        pca = np.transpose(Rtc)*(Rt0*(p0a - p0t) - ptc)
        pca = pca + rng.normal(0,0.001,[3,1]) # add noise
        # add to lists
        angle_list.append(angles)
        pca_list.append(pca)

    # Estimate transformation
    (p_est,R_est) = get_pose(angle_list,pca_list)
    print "Percent errors"
    print "pct:"
    print (p_est-ptc)/ptc
    print "Rct:"
    print (R_est-Rtc)/Rtc
