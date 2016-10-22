import numpy as np

def rotz(th):
    c,s = np.cos(th),np.sin(th)
    return np.matrix([[c,-s,0],[s,c,0],[0,0,1]])

def roty(th):
    c,s = np.cos(th),np.sin(th)
    return np.matrix([[c,0,s],[0,1,0],[-s,0,c]])

def rotx(th):
    c,s = np.cos(th),np.sin(th)
    return np.matrix([[1,0,0],[0,c,-s],[0,s,c]])
