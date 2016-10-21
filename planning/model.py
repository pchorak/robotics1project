# Model of the dobot arm to be used for collision detection
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#import itertools as it

import intersect
import kinematics

plt.ion()

# Arm parameters
l1 = 135
l2 = 160
d = 50

# Vertices
#it.product([-20,10],[-5,5],[0,135])
link1 = np.transpose(np.matrix([[10.0,-5,0],[-20,-5,0],[10,5,0],[-20,5,0], \
    [10,-5,l1],[-20,-5,l1],[10,5,l1],[-20,5,l1]]))

link2 = np.transpose(np.matrix([[10.0,-5,0],[-20,-5,0],[10,5,0],[-20,5,0], \
    [10,-5,l2],[-20,-5,l2],[10,5,l2],[-20,5,l2]]))

# end effector
hand = np.transpose(np.matrix([[15.0,-5,0],[-15,-5,0],[15,5,0],[-15,5,0], \
    [15,-5,d],[-15,-5,d],[15,5,d],[-15,5,d]]))

# Faces defined relative to concatonated vertex list
tube = np.array([[0,1,4],[1,4,5],[1,3,5],[3,5,7],[3,2,7],[2,7,6],[2,0,6],[0,6,4]])
faces = np.vstack(([[0,1,3],[0,2,3]],tube,tube+8,tube+16,[[20,21,23],[20,22,23]]))

obstacles = np.array([[[230.0,-10,0],[230,10,0],[240,0,200]], \
    [[230,-10,0],[250,-10,0],[240,0,200]], \
    [[230,10,0],[250,10,0],[240,0,200]], \
    [[250,-10,0],[250,10,0],[240,0,200]], \
    [[-50,-200,-30],[-50, 200,-30],[350,200,-30]], \
    [[-50,-200,-30],[350,-200,-30],[350,200,-30]]])

def collision(angles):
    """
    Returns whether the arm model intersects any obstacles in a given configuration (psi,th1,th2).
    """
    arm = _transform(angles)

    # Check for collisions
    for Ta in arm:
        for To in obstacles:
            if intersect.triangles(Ta,To):
                return True
    return False

def display(angles):
    """
    Plots wireframe models of the arm and obstacles.
    """
    arm = _transform(angles)

    fig = plt.gcf()#figure(1)
    ax = Axes3D(fig)
    #plt.axis('equal')
    for Ta in arm:
        ax.plot(Ta[[0,1,2,0],0],Ta[[0,1,2,0],1],Ta[[0,1,2,0],2],'b')
    for To in obstacles:
        ax.plot(To[[0,1,2,0],0],To[[0,1,2,0],1],To[[0,1,2,0],2],'b')
    plt.xlim([-50,350])
    plt.ylim([-200,200])
    plt.show()

def plot_path(path,qs):
    """
    Plots the position of the end effector for a list of configurations "qs"
    and draws lines along the path described by the configurations "path".
    """
    path = np.array([kinematics.forward(q) for q in path])
    ps = np.array([kinematics.forward(q) for q in qs])

    fig = plt.gcf()
    ax = Axes3D(fig)
    for To in obstacles:
        ax.plot(To[[0,1,2,0],0],To[[0,1,2,0],1],To[[0,1,2,0],2],'b')
    ax.plot(path[:,0],path[:,1],path[:,2],'r')
    ax.plot(ps[:,0],ps[:,1],ps[:,2],'.g')
    plt.xlim([-50,350])
    plt.ylim([-200,200])
    plt.show()

def _transform(angles):
    """
    Transforms the arm model into a single triangle mesh in the global reference frame.
    """
    psi = angles[0]
    th1 = angles[1]
    th2 = angles[2]

    # Transform the model components into the global reference frame
    phi = np.pi/2 - (th1 + th2)
    R0 = rotz(psi)
    R1 = roty(th1)
    R2 = roty(th2)
    R3 = roty(phi)
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

def rotz(th):
    c,s = np.cos(th),np.sin(th)
    return np.matrix([[c,-s,0],[s,c,0],[0,0,1]])

def roty(th):
    c,s = np.cos(th),np.sin(th)
    return np.matrix([[c,0,s],[0,1,0],[-s,0,c]])

def rotx(th):
    c,s = np.cos(th),np.sin(th)
    return np.matrix([[1,0,0],[0,c,-s],[0,s,c]])
