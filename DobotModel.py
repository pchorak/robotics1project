# Kinematics module for the dobot
import numpy as np
import numpy.testing as npt

import math3D

# Arm parameters
l1 = 135 # length of arm link 1
l2 = 160 # length of arm link2 2
d = 50 # length of end effector

# Vertices
#import itertools as it
#it.product([-20,10],[-5,5],[0,135])
link1 = np.transpose(np.matrix([[10.0,-5,0],[-20,-5,0],[10,5,0],[-20,5,0], \
    [10,-5,l1],[-20,-5,l1],[10,5,l1],[-20,5,l1]]))
link2 = np.transpose(np.matrix([[10.0,-5,0],[-20,-5,0],[10,5,0],[-20,5,0], \
    [10,-5,l2],[-20,-5,l2],[10,5,l2],[-20,5,l2]]))
hand = np.transpose(np.matrix([[15.0,-5,0],[-15,-5,0],[15,5,0],[-15,5,0], \
    [15,-5,d],[-15,-5,d],[15,5,d],[-15,5,d]]))

# Faces defined relative to concatonated vertex list
tube = np.array([[0,1,4],[1,4,5],[1,3,5],[3,5,7],[3,2,7],[2,7,6],[2,0,6],[0,6,4]])
faces = np.vstack(([[0,1,3],[0,2,3]],tube,tube+8,tube+16,[[20,21,23],[20,22,23]]))

# Precompute some quantities
l1_sq = l1**2
l2_sq = l2**2

pi2 = np.pi/2
pi4 = np.pi/4
d2r = np.pi/180 # degrees to radians

def forward_kinematics(angles):
    """
    Input: (psi,th1,th2)
        psi - angle about base
        th1 - angle of first link from verticle
        th2 - angle of second link relative to first
    Output:
        v - x,y,z coordinate of the end effector (3,)
    """
    psi = angles[0]
    th1 = angles[1]
    th2 = angles[2]
    r = l1*np.sin(th1) + l2*np.sin(th1 + th2) + d
    v = np.zeros(3)
    v[0] = r*np.cos(psi) # x
    v[1] = r*np.sin(psi) # y
    v[2] = l1*np.cos(th1) + l2*np.cos(th1 + th2) # z
    return v

def jacobian(angles):
    """
    Input: (psi,th1,th2)
    Output: Jacobian matrix d(x,y,z)/d(psi,th1,th2)
    """
    psi = angles[0]
    th1 = angles[1]
    th2 = angles[2]

    s0 = np.sin(psi)
    c0 = np.cos(psi)
    s1 = np.sin(th1)
    c1 = np.cos(th1)
    s12 = np.sin(th1+th2)
    c12 = np.cos(th1+th2)
    r = l1*s1 + l2*s12 + d

    return np.matrix([[(l1*c1+l2*c12)*c0,l2*c12*c0,-r*s0], \
        [(l1*c1+l2*c12)*s0,l2*c12*s0,r*c0], \
        [-(l1*s1+l2*s12),-l2*s12,0]])
    
def inverse_kinematics(v):
    """
    Input:
        v - x,y,z coordinate of the end effector (3,)
    Output: (psi,th1,th2,phi)
        psi - angle about base
        th1 - angle of first link from verticle
        th2 - angle of second link relative to first
        phi - angle of wrist (to make the effector level)
    """
    x = v[0]
    y = v[1]
    z = v[2]
    # pre-compute distances
    r = np.sqrt(x**2 + y**2) # radial position of end effector in x-y plane
    c_sq = (r - d)**2 + z**2
    c = np.sqrt(c_sq) # distance of the links joined at the elbow

    # law of cosines
    alpha = np.arccos((l1_sq + c_sq - l2_sq)/(2*l1*c))
    gamma = np.arccos((l1_sq + l2_sq - c_sq)/(2*l1*l2))

    # joint angles
    psi = np.arctan2(y,x)
    th1 = pi2 - np.arctan2(z,r-d) - alpha
    th2 = np.pi - gamma
    phi = pi2 - (th1 + th2) # always orient the end effector parallel with the x-y plane

    # return nan for the angles if the position is unreachable
    nan = np.nan
    if np.isnan(psi) or np.isnan(th1) or np.isnan(th2):
        return (nan,nan,nan,nan)
    elif (psi < -135*d2r or 135*d2r < psi):
        return (nan,nan,nan,nan)
    elif (th1 < -5*d2r or 85*d2r < th1):
        return (nan,nan,nan,nan)
    elif (th2 < -10*d2r or 95*d2r < th2):
        return (nan,nan,nan,nan)
    elif (phi < -90*d2r or 90*d2r < phi):
        return (nan,nan,nan,nan)
    else:
        return (psi,th1,th2,phi)


def _transform(angles):
    """
    Transforms the arm model into a single triangle mesh in the global reference frame.
    """
    psi = angles[0]
    th1 = angles[1]
    th2 = angles[2]

    # Transform the model components into the global reference frame
    phi = np.pi/2 - (th1 + th2)
    R0 = math3D.rotz(psi)
    R1 = math3D.roty(th1)
    R2 = math3D.roty(th2)
    R3 = math3D.roty(phi)
    p1 = np.matrix([[0],[0],[l1]])
    p2 = np.matrix([[0],[0],[l2]])
    verts = np.transpose(np.array(np.hstack((R0*R1*link1, \
        R0*R1*(R2*link2+np.tile(p1,(1,link2.shape[1]))), \
        R0*R1*(R2*(R3*hand+np.tile(p2,(1,hand.shape[1])))+np.tile(p1,(1,hand.shape[1])))))))

    # Construct the array of triangles (arrays of vertices)
    arm = np.zeros([len(faces),3,3])
    for k in xrange(len(faces)):
        arm[k] = verts[faces[k]]
    return arm

def test():
    # test forward kinematics
    npt.assert_array_almost_equal(forward((0,0,0)),[d,0,l1+l2])
    npt.assert_array_almost_equal(forward((0,pi2,0)),[l1+l2+d,0,0])
    npt.assert_array_almost_equal(forward((0,pi4,0)),np.array([l1+l2,0,l1+l2])/np.sqrt(2) + [d,0,0])
    npt.assert_array_almost_equal(forward((0,0,pi2)),[l2+d,0,l1])
    npt.assert_array_almost_equal(forward((0,0,pi4)),[d+l2/np.sqrt(2),0,l1+l2/np.sqrt(2)])
    npt.assert_array_almost_equal(forward((pi2,pi2,0)),[0,l1+l2+d,0])
    npt.assert_array_almost_equal(forward((pi4,pi2,0)),np.array([l1+l2+d,l1+l2+d,0])/np.sqrt(2))

    # test inverse kinematics
    npt.assert_array_almost_equal(inverse(forward((0,0,0))),(0,0,0,pi2))
    npt.assert_array_almost_equal(inverse(forward((0,0.9*pi4,0))),(0,0.9*pi4,0,1.1*pi4))
    npt.assert_array_almost_equal(inverse(forward((0,pi4,0))),(0,pi4,0,pi4))
    npt.assert_array_almost_equal(inverse(forward((0,0,pi2))),(0,0,pi2,0))
    npt.assert_array_almost_equal(inverse(forward((0,0,pi4))),(0,0,pi4,pi4))
    npt.assert_array_almost_equal(inverse(forward((pi2,pi4,0))),(pi2,pi4,0,pi4))
    npt.assert_array_almost_equal(inverse(forward((pi4,pi4,0))),(pi4,pi4,0,pi4))
