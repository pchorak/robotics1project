# Model of the dobot arm to be used for collision detection
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import intersect
import DobotModel

plt.ion()

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
    arm = DobotModel.get_mesh(angles)

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
    arm = DobotModel.get_mesh(angles)

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
